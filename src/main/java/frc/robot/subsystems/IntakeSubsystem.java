// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MathR;

public class IntakeSubsystem extends SubsystemBase{

  private final int TILT_TOLERANCE = 2;

  private PIDController tiltPID = new PIDController(0.008, 0, 0);

  private TalonFX intakeSpinnerMotor = new TalonFX(Constants.INTAKE_SPINNER_ID);
  private static TalonFX intakeTiltMotor = new TalonFX(Constants.INTAKE_PIVOT_ID);
  
  private static DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(Constants.INTAKE_ENCODER_DIGITAL_PORT_A);


  private IntakePosition currentSetPosition = IntakePosition.RETRACTED;
  private double speedLimit = 0.2;

  public static final double MAX_DEGREES = 135;
  public static final double MIN_DEGREES = 0;

  public IntakeSubsystem() {
    //Set intake tilt tolerance
    tiltPID.setTolerance(TILT_TOLERANCE);
  }

  //Get intake pitch angle
  public static double getPitch(){
    return MathR.getDistanceToAngle(0, tiltEncoder.getAbsolutePosition() / (Constants.INTAKE_TILT_ENCODER_MAX_VALUE - Constants.INTAKE_TILT_ENCODER_MIN_VALUE) * 360 + Constants.INTAKE_TILT_ENCODER_OFFSET, 180);
  }

  //Tilt the intake to a certain angle
  public void tiltToAngle(double degrees){
    double power = MathR.limit(tiltPID.calculate(MathR.getDistanceToAngle(getPitch(), degrees), 0), -1, 1);

    intakeTiltMotor.setControl(new DutyCycleOut(power, true, false, false, false));
    //intakeTiltMotor.set(MathR.limit(tiltPID.calculate(MathR.getDistanceToAngle(getPitch(), degrees), 0), -1, 1));
  }

  //Set the intake speed
  public void setIntake(double speed){
    intakeSpinnerMotor.setControl(new DutyCycleOut(speed, true, false, false, false));
  }

  //Set the intake tilt to a certain speed
  public void set(double speed) {
    currentSetPosition = IntakePosition.MANUAL;

    intakeTiltMotor.set(speed);
    /*intakeTiltMotor.set(
        MathR.limitWhenReached(speed, -speedLimit, speedLimit, getPitch() <= MIN_DEGREES,
            getPitch() >= MAX_DEGREES));*/
  }

  //Set the intake to a certain named position
  public void set(IntakePosition pos) {
    tiltPID.setSetpoint(0);
    double speed = -MathR.limit(tiltPID.calculate(MathR.getDistanceToAngle(getPitch(), pos.angle)), -speedLimit, speedLimit);
   
    
    if (speed > 0 && getPitch() >= MAX_DEGREES){
      speed = 0;
    }
    else if (speed < 0 && getPitch() <= MIN_DEGREES){
      speed = 0;
    }

    
    if (!atSetPosition())
      set(speed);
    else
      set(0.0);

    currentSetPosition = pos;
  }

  //Run the intake tilt motor
  public void setManual(double speed) {
    intakeTiltMotor.set(speed);
  }

  //Check if the intake is at a certain position within a tolerance
  public boolean atSetPosition() {
    return tiltPID.atSetpoint();
  }

  //Return the intake's current set position
  public IntakePosition getSetPosition() {
    return currentSetPosition;
  }

  //Set the intake tilt speed limit
  public void setSpeedLimit(double max) {
    speedLimit = max;
  }

  //Intake angle names
  public enum IntakePosition {
    RETRACTED(134),
    EXTENDED(1),
    OUT_OF_THE_WAY(110),
    RUNNING_EXTENDED(2),
    MANUAL(-1);

    public final double angle;
    private IntakePosition(double angle) {
      this.angle = angle;
    }
  }

  @Override
  public void periodic() {
    
    
  }
}
