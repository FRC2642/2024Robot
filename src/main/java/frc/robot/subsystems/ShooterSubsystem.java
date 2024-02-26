// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.IPositionable;
import frc.robot.utils.MathR;

public class ShooterSubsystem extends SubsystemBase implements IPositionable<ShooterSubsystem.ShooterPosition>{

  private PIDController tiltPID = new PIDController(0.05, 0, 0);
  private PIDController shooterPID = new PIDController(0.2, 0, 0);
  
  private TalonFX shooterMotor = new TalonFX(Constants.SHOOTER_SPINNER_ID);
  private CANSparkMax shooterTiltMotor = new CANSparkMax(Constants.SHOOTER_PIVOT_ID, MotorType.kBrushless);
  private CANSparkMax feederMotor = new CANSparkMax(Constants.FEEDER_WHEELS_ID, MotorType.kBrushless);

  private static SparkAbsoluteEncoder tiltEncoder;

  private DigitalInput beamBreak = new DigitalInput(Constants.BEAM_BREAK_CHANNEL);

  private ShooterPosition currentSetPosition = ShooterPosition.TRAVEL;
  private double speedLimit = 0.2;

  public static final double MAX_DEGREES = 90;
  public static final double MIN_DEGREES = -40;
  private final int TILT_TOLERANCE = 4;
  
  public ShooterSubsystem() {
    tiltEncoder = shooterTiltMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    tiltPID.setTolerance(TILT_TOLERANCE);
  }

  public double getPitch(){
    return MathR.getDistanceToAngle(0, tiltEncoder.getPosition() / (Constants.SHOOTER_TILT_ENCODER_MAX_VALUE - Constants.SHOOTER_TILT_ENCODER_MIN_VALUE) * 360 + Constants.SHOOTER_TILT_ENCODER_OFFSET, 180);
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
    tiltPID.setSetpoint(0);
    double speed = tiltPID.calculate(MathR.getDistanceToAngle(getPitch(), pos.angle));

    /*if (speed < 0 && pos.angle < 0 && ElevatorSubsystem.getPercentElevated() <= 0.2){
      speed = 0;
    }*/

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

  public void setShooterRPM(){
    double currentRPM = shooterMotor.getVelocity().getValue();
    setShooter(shooterPID.calculate(currentRPM, Constants.SHOOTER_SET_RPM));
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
    TRAVEL(17.5),
    AMP(-20),
    TRAP(45),
    MANUAL(-1);

    public final double angle;
    private ShooterPosition(double angle) {
      this.angle = angle;
    }
  }

  @Override
  public void periodic() {}
}
