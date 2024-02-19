// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.IPositionable;
import frc.robot.utils.MathR;

public class ShooterSubsystem extends SubsystemBase implements IPositionable<ShooterSubsystem.ShooterPosition>{

  private PIDController tiltPID = new PIDController(0.2, 0, 0);
  
  private TalonFX shooterMotor = new TalonFX(Constants.SHOOTER_SPINNER_ID);
  private CANSparkMax shooterTiltMotor = new CANSparkMax(Constants.SHOOTER_PIVOT_ID, MotorType.kBrushless);
  private CANSparkMax feederMotor = new CANSparkMax(Constants.FEEDER_WHEELS_ID, MotorType.kBrushless);

  private AbsoluteEncoder tiltEncoder = shooterTiltMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private DigitalInput beamBreak = new DigitalInput(Constants.BEAM_BREAK_CHANNEL);

  private ShooterPosition currentSetPosition = ShooterPosition.TRAVEL;
  private double speedLimit = 1;

  public static final double MAX_DEGREES = 90;
  public static final double MIN_DEGREES = -40;
  private final int TILT_TOLERANCE = 4;
  
  public ShooterSubsystem() {
    tiltPID.setTolerance(TILT_TOLERANCE);
  }

  public double getPitch(){
    return tiltEncoder.getPosition() * Constants.SHOOTER_TILT_ENCODER_TICKS_PER_DEGREE;
  }

  public boolean getNoteDetected(){
    return !beamBreak.get();
  }

  public void tiltToAngle(double degrees){
    shooterTiltMotor.set(MathR.limit(tiltPID.calculate(MathR.getDistanceToAngle(getPitch(), degrees), 0), -1, 1));
  }

  public void set(double speed) {
    currentSetPosition = ShooterPosition.MANUAL;

    shooterTiltMotor.set(
        MathR.limitWhenReached(speed, -speedLimit, speedLimit, getPitch() <= MIN_DEGREES,
            getPitch() >= MAX_DEGREES));
  }

  public void set(ShooterPosition pos) {
    double speed = tiltPID.calculate(getPitch(), pos.angle);

    if (!atSetPosition())
      set(speed);
    else
      set(0.0);

    currentSetPosition = pos;
  }

  public void setManual(double speed) {
    shooterTiltMotor.set(speed);
  }

  public void setFeeder(double speed){
    feederMotor.set(speed);
  }

  public void setShooter(double speed){
    shooterMotor.set(speed);
  }


  @Override
  public boolean atSetPosition() {
    return tiltPID.atSetpoint();
  }

  @Override
  public ShooterPosition getSetPosition() {
    return currentSetPosition;
  }

  @Override
  public void setSpeedLimit(double max) {
    speedLimit = max;
  }

  @Override
  public double getSpeedLimit() {
    return speedLimit;
  }

  public void setRampRate(double rampRate) {
    shooterTiltMotor.setOpenLoopRampRate(rampRate);
  }

  public double getRampRate() {
    return shooterTiltMotor.getOpenLoopRampRate();
  }

  public enum ShooterPosition {
    TRAVEL(60),
    AMP(-20),
    TRAP(50),
    MANUAL(-1);

    public final double angle;
    private ShooterPosition(double angle) {
      this.angle = angle;
    }
  }

  @Override
  public void periodic() {}
}
