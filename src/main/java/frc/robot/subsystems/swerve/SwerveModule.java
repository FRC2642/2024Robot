// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

/**
 * Thic class represents a single swerve module. It allows for a desired speed
 * and angle to be set. Motor controllers are interfaced with directly from the
 * update() method.
 * 
 * To be noted, this is the main class for handeling each drive. 
 * SwerveModules and SwerveModuleInfo only organize the types and 
 * variables of the stuff here to be used elsewhere.
 */
public class SwerveModule {

  // HARDWARE
  public final TalonFX angleMotor;
  private final TalonFX driveMotor;
  public final CANCoder orientationEncoder;

  // INFORMATION
  public final SwerveModuleInfo info;
  private final double defensiveAngleDeg;
  private double wheelOrientation = 0.0;
  

  public SwerveModule(SwerveModuleInfo info) {
    this.info = info;
    this.angleMotor = new TalonFX(info.TURN_ID);
    this.driveMotor = new TalonFX(info.DRIVE_ID);
    this.orientationEncoder = new CANCoder(info.ENCODER_ID);
    this.defensiveAngleDeg = VectorR.fromCartesian(info.X, info.Y).getAngle();
    orientationEncoder.setPosition(0);
  }


  // MODULE WHEEL MEASUREMENTS
  public double getWheelSpeed() {
    return driveMotor.getVelocity().getValue() * Constants.FEET_PER_DISPLACEMENT; /* * (100d/1d);*/
  }

  private double getWheelPosition() {
    return driveMotor.getPosition().getValue() * Constants.FEET_PER_DISPLACEMENT;
  }

  /*
   * positive (+) = left turn CCW
   * negative (-) = right turn CW
   * Does that make sense to you?
   */
  public double getWheelOrientationDegrees() {
    return wheelOrientation- info.ABS_ENCODER_VALUE_WHEN_STRAIGHT;
  }

  public VectorR getVelocity() { //Get how fast the wheel's going (?)
    return VectorR.fromPolar(getWheelSpeed(), getWheelOrientationDegrees());
  }

  private double lastWheelPosition = 0;
  private double increment = 0;

  public VectorR getPositionIncrement() { 
    return VectorR.fromPolar(increment, getWheelOrientationDegrees());
  }
  
  private void updateIncrementMeasurement() {
    double pos = getWheelPosition();
    increment = pos - lastWheelPosition;
    lastWheelPosition = pos;
  }

  // MODULE SPEEDS CALCULATIONS
  private VectorR desired = new VectorR();
  private boolean reversed = false;

  private void reverse() { //Reverse the robot
    reversed = !reversed;
  }

  private double desiredSpeed() { //Get how fast you want the wheel to go, for PID shenanigans
    if (reversed)
      return desired.getTerminalMagnitude();
    else
      return desired.getMagnitude();
  }

  private double desiredAngle() { //Gets the angle we want the wheel to be turned at
    if (reversed)
      return desired.getTerminalAngle();
    else
      return desired.getAngle();
  }

  /*
   * UPDATE OR STOP METHODS MUST BE CALLED PERIODICALLY 
   * speed 0 min - 1 max, turns module drive wheel
   * angle radians follows coordinate plane standards, sets module wheel to angle
   */
  public void update(double speed, double angleDegrees) {
    wheelOrientation = orientationEncoder.getAbsolutePosition();
    desired.setFromPolar(speed, angleDegrees);

    if (Math.abs(MathR.getDistanceToAngle(getWheelOrientationDegrees(), desiredAngle())) > 90d)
      reverse();
    
    double speed_power = MathR.limit(desiredSpeed(), -1.0, 1.0);
    double angle_power = MathR
        .limit(Constants.MODULE_ANGLE_KP * MathR.getDistanceToAngle(getWheelOrientationDegrees(), desiredAngle()), -1, 1);
  
    
    driveMotor.setControl(new DutyCycleOut(speed_power, true, false, false, false));
    angleMotor.setControl(new DutyCycleOut(angle_power, true, false, false, false));
    //driveMotor.set(speed_power); 
    //angleMotor.set(angle_power);

    updateIncrementMeasurement();
  }

  public void stop() { //Stop the wheels
    angleMotor.set(0);
    driveMotor.set(0);
    
    
    updateIncrementMeasurement();
  }

  public void stopDefensively() { //Since we're using swerve, turning the wheels in a diamond pattern stops the bot from moving entirely
    update(0.0000001,  defensiveAngleDeg);
  }
}

/* ~~A very short nessecary crash course on Swerve~~
 * 
 * In a swerve drive robot, each wheel has two motors, one controlling the wheel itself
 * and the other controlling the direction the wheel is facing. Each of these motors operates
 * independently, meaning that you can move in any direction, pivot, defend, and more just by
 * reorienting each wheel, which can be turned 360 degrees.
 * 
 * -   -   |    |   /    /    In order to drive in a single direction, the wheels must all be
 *                            oriented in the same direction. By changing all of the wheel
 * -   -   |    |   /    /    directions together, you can drive the robot without turning.
 * 
 * \   /      /   \    These two positions are useful for specific scenarios. The X pattern
 *                     stops the robot where it is and prevents it from being moved by others.
 * /   \      \   /    The diamond pattern allows the robot to pivot in place
 * 
 *   =   Note that each wheel only needs to rotate 180 degrees, not 360. If you reverse the
 *  \\   direction of the drive motors, it willbe able to drive the opposite direction of the
 *  //   180 degrees you can go forward, saving the robot time. To drive a certain direction,
 *  ||   the wheels should optimally rotate no more than 90 degrees in any direction.
 * 
 */
