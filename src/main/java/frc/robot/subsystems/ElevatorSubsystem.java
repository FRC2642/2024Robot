// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.IPositionable;
import frc.robot.utils.MathR;

public class ElevatorSubsystem extends SubsystemBase implements IPositionable<ElevatorSubsystem.ElevatorPosition>{

  private PIDController elevatorPID = new PIDController(0.2, 0, 0);

  private TalonFX elevatorMotor1 = new TalonFX(Constants.ELEVATOR_MOTOR_1_ID);
  private TalonFX elevatorMotor2 = new TalonFX(Constants.ELEVATOR_MOTOR_2_ID);
  
  private AnalogInput elevatorEncoder = new AnalogInput(Constants.ELEVATOR_ENCODER_ANALOG_PORT);

  private ElevatorPosition currentSetPosition = ElevatorPosition.TRAVEL;
  private double speedLimit = 0.8;
  private final double PID_TOLERANCE = 0.05;

  public ElevatorSubsystem() {
    elevatorPID.setTolerance(PID_TOLERANCE);
  }

  public double getHeight(){
    return elevatorEncoder.getValue() * Constants.ELEVATOR_MAX_HEIGHT_FEET;
  }

  public double getPercentElevated(){
    return elevatorEncoder.getValue() / Constants.ELEVATOR_MAX_ENCODER_TICK;
  }

  public void moveToPosition(double percentElevated){
    double power = MathR.limit(elevatorPID.calculate(getPercentElevated(), percentElevated), -1, 1);
    elevatorMotor1.set(power);
    elevatorMotor2.set(power);

  }

  public void set(double speed) {
    currentSetPosition = ElevatorPosition.MANUAL;

    double power = MathR.limitWhenReached(speed, -speedLimit, speedLimit, getPercentElevated() <= 0, getPercentElevated() >= 1);

    elevatorMotor1.set(power);
    elevatorMotor2.set(power);
  }

  public void set(ElevatorPosition pos) {
    double speed = elevatorPID.calculate(getPercentElevated(), pos.percentageElevated);

    if ((Math.abs(speed) > 0 && IntakeSubsystem.getPitch() >= 70)) {
      speed = 0.0;
    }

    if (!atSetPosition())
      set(speed);
    else
      set(0.0);

    currentSetPosition = pos;
  }

  public void setManual(double speed) {
    elevatorMotor1.set(speed);
    elevatorMotor2.set(speed);
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
