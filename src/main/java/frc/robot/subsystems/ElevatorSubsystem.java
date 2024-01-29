// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IPositionable;
import frc.robot.subsystems.ShooterSubsystem.ShooterPosition;
import frc.robot.utils.MathR;

public class ElevatorSubsystem extends SubsystemBase implements IPositionable<ElevatorSubsystem.ElevatorPosition>{

  private PIDController elevatorPID = new PIDController(0.2, 0, 0);

  private CANSparkMax elevatorMotor1 = new CANSparkMax(Constants.ELEVATOR_MOTOR_1_ID, MotorType.kBrushless);
  private CANSparkMax elevatorMotor2 = new CANSparkMax(Constants.ELEVATOR_MOTOR_2_ID, MotorType.kBrushless);
  private AbsoluteEncoder elevatorEncoder = elevatorMotor1.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);

  private ElevatorPosition currentSetPosition = ElevatorPosition.TRAVEL;
  private double speedLimit = 0.8;

  public ElevatorSubsystem() {
    elevatorMotor2.follow(elevatorMotor1);
    elevatorEncoder.setPositionConversionFactor(1/Constants.ELEVATOR_MAX_ENCODER_TICK);
  }

  public double getHeight(){
    return elevatorEncoder.getPosition() * Constants.ELEVATOR_MAX_HEIGHT;
  }

  public double getPercentElevated(){
    return elevatorEncoder.getPosition();
  }

  public void moveToPosition(double percentElevated){
    elevatorMotor1.set(MathR.limit(elevatorPID.calculate(getPercentElevated(), percentElevated), -1, 1));
  }

  public void set(double speed) {
    currentSetPosition = ElevatorPosition.MANUAL;

    elevatorMotor1.set(
        MathR.limitWhenReached(speed, -speedLimit, speedLimit, getPercentElevated() <= 0,
            getPercentElevated() >= 1));
  }

  public void set(ElevatorPosition pos) {
    double speed = elevatorPID.calculate(getPercentElevated(), pos.percentageElevated);

    if (!atSetPosition())
      set(speed);
    else
      set(0.0);

    currentSetPosition = pos;
  }

  public void setManual(double speed) {
    elevatorMotor1.set(speed);
  }

  @Override
  public boolean atSetPosition() {
    return elevatorPID.atSetpoint();
  }

  @Override
  public ElevatorPosition getSetPosition() {
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

  @Override
  public void setRampRate(double rampRate) {
    elevatorMotor1.setOpenLoopRampRate(rampRate);
  }

  @Override
  public double getRampRate() {
    return elevatorMotor1.getOpenLoopRampRate();
  }

  public enum ElevatorPosition {
    TRAVEL(0),
    AMP(0.5),
    TRAP(0.9),
    CLIMB(1),
    MANUAL(-1);

    public final double percentageElevated;
    private ElevatorPosition(double percentageElevated) {
      this.percentageElevated = percentageElevated;
    }
  }

  @Override
  public void periodic() {}
}
