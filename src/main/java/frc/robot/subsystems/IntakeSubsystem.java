// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.IPositionable;
import frc.robot.utils.MathR;

public class IntakeSubsystem extends SubsystemBase implements IPositionable<IntakeSubsystem.IntakePosition>{

  private final int TILT_TOLERANCE = 6;

  private PIDController tiltPID = new PIDController(0.01, 0, 0);

  private TalonFX intakeSpinnerMotor = new TalonFX(Constants.INTAKE_SPINNER_ID);
  private static CANSparkMax intakeTiltMotor = new CANSparkMax(Constants.INTAKE_PIVOT_ID, MotorType.kBrushless);
  
  private static SparkAbsoluteEncoder tiltEncoder;

  private IntakePosition currentSetPosition = IntakePosition.RETRACTED;
  private double speedLimit = 0.2;

  public static final double MAX_DEGREES = 152;
  public static final double MIN_DEGREES = -10;

  public IntakeSubsystem() {
    tiltEncoder = intakeTiltMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    tiltPID.setTolerance(TILT_TOLERANCE);
  }

  public static double getPitch(){
    return MathR.getDistanceToAngle(0, tiltEncoder.getPosition() / (Constants.INTAKE_TILT_ENCODER_MAX_VALUE - Constants.INTAKE_TILT_ENCODER_MIN_VALUE) * 360 + Constants.INTAKE_TILT_ENCODER_OFFSET, 180);
  }

  public void tiltToAngle(double degrees){
    intakeTiltMotor.set(MathR.limit(tiltPID.calculate(MathR.getDistanceToAngle(getPitch(), degrees), 0), -1, 1));
  }

  public void setIntake(double speed){
    intakeSpinnerMotor.set(speed);
  }

  public void set(double speed) {
    currentSetPosition = IntakePosition.MANUAL;

    intakeTiltMotor.set(speed);
    /*intakeTiltMotor.set(
        MathR.limitWhenReached(speed, -speedLimit, speedLimit, getPitch() <= MIN_DEGREES,
            getPitch() >= MAX_DEGREES));*/
  }

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

  public void setManual(double speed) {
    intakeTiltMotor.set(speed);
  }

  public double getPower(){
    return intakeSpinnerMotor.get();
  }

  public double getVelocity(){
    return intakeSpinnerMotor.getVelocity().getValueAsDouble();
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

  public enum IntakePosition {
    RETRACTED(133),
    EXTENDED(0),
    OUT_OF_THE_WAY(110),
    AMP(105),
    RUNNING_EXTENDED(2),
    MANUAL(-1);

    public final double angle;
    private IntakePosition(double angle) {
      this.angle = angle;
    }
  }

  @Override
  public void periodic() {}
}
