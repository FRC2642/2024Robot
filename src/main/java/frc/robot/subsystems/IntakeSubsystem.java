// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.IPositionable;
import frc.robot.utils.MathR;

public class IntakeSubsystem extends SubsystemBase implements IPositionable<IntakeSubsystem.IntakePosition>{

  private final int TILT_TOLERANCE = 3;

  private PIDController tiltPID = new PIDController(0.2, 0, 0);

  private CANSparkMax intakeSpinnerMotor = new CANSparkMax(Constants.INTAKE_SPINNER_ID, MotorType.kBrushless);
  private CANSparkMax intakeTiltMotor = new CANSparkMax(Constants.INTAKE_PIVOT_ID, MotorType.kBrushless);
  
  private AbsoluteEncoder tiltEncoder = intakeTiltMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private IntakePosition currentSetPosition = IntakePosition.RETRACTED;
  private double speedLimit = 0.5;

  public static final double MAX_DEGREES = 80;
  public static final double MIN_DEGREES = 0;

  public IntakeSubsystem() {
    tiltPID.setTolerance(TILT_TOLERANCE);
  }

  public double getPitch(){
    return tiltEncoder.getPosition() * Constants.INTAKE_TILT_ENCODER_TICKS_PER_DEGREE;
  }

  public void tiltToAngle(double degrees){
    intakeTiltMotor.set(MathR.limit(tiltPID.calculate(MathR.getDistanceToAngle(getPitch(), degrees), 0), -1, 1));
  }

  public void setIntake(double speed){
    intakeSpinnerMotor.set(speed);
  }

  public void set(double speed) {
    currentSetPosition = IntakePosition.MANUAL;

    intakeTiltMotor.set(
        MathR.limitWhenReached(speed, -speedLimit, speedLimit, getPitch() <= MIN_DEGREES,
            getPitch() >= MAX_DEGREES));
  }

  public void set(IntakePosition pos) {
    double speed = tiltPID.calculate(getPitch(), pos.angle);

    if (!atSetPosition())
      set(speed);
    else
      set(0.0);

    currentSetPosition = pos;
  }

  public void setManual(double speed) {
    intakeTiltMotor.set(speed);
  }

  @Override
  public boolean atSetPosition() {
    return tiltPID.atSetpoint();
  }

  @Override
  public IntakePosition getSetPosition() {
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
    intakeTiltMotor.setOpenLoopRampRate(rampRate);
  }

  public double getRampRate() {
    return intakeTiltMotor.getOpenLoopRampRate();
  }

  public enum IntakePosition {
    RETRACTED(0),
    EXTENDED(0),
    MANUAL(-1);

    public final double angle;
    private IntakePosition(double angle) {
      this.angle = angle;
    }
  }

  @Override
  public void periodic() {}
}
