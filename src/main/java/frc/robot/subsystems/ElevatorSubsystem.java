// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.IPositionable;
import frc.robot.utils.MathR;

public class ElevatorSubsystem extends SubsystemBase implements IPositionable<ElevatorSubsystem.ElevatorPosition>{

  private PIDController elevatorPID = new PIDController(/*0.01*/0.1, 0, 0);

  private TalonFX elevatorMotor1 = new TalonFX(Constants.ELEVATOR_MOTOR_1_ID);
  private TalonFX elevatorMotor2 = new TalonFX(Constants.ELEVATOR_MOTOR_2_ID);
  
  private static Encoder elevatorEncoder = new Encoder(Constants.ELEVATOR_ENCODER_DIGITAL_PORT_A, Constants.ELEVATOR_ENCODER_DIGITAL_PORT_B);

  private ElevatorPosition currentSetPosition = ElevatorPosition.TRAVEL;
  private double speedLimit = 0.5;
  private final double PID_TOLERANCE = 0.10;

  public ElevatorSubsystem() {
    elevatorPID.setTolerance(PID_TOLERANCE);
  }

  public double getHeight(){
    return 0;
    //return elevatorEncoder.getRaw() * Constants.ELEVATOR_MAX_HEIGHT_FEET;
  }

  public static double getPercentElevated(){
    
    return -elevatorEncoder.getRaw() / Constants.ELEVATOR_ENCODER_WHEN_AT_TOP + Constants.ELEVATOR_ENCODER_OFFSET;
  }

  public static void resetEncoder(){
    elevatorEncoder.reset();
  }

  public void set(double speed) {
    currentSetPosition = ElevatorPosition.MANUAL;

    double power = MathR.limitWhenReached(speed, -speedLimit, speedLimit, getPercentElevated() <= 0, getPercentElevated() >= 1);

    
    elevatorMotor1.set(speed);
    elevatorMotor2.set(-speed);
  }

  public void set(ElevatorPosition pos) {
    double speed = elevatorPID.calculate(getPercentElevated(), pos.percentageElevated);
    
    /*if ((Math.abs(speed) > 0 && IntakeSubsystem.getPitch() >= 70)) {
      speed = 0.0;
    }*/

    if (!atSetPosition()){
      set(speed);
    }
    else{
      set(0.0);
    }
      

    currentSetPosition = pos;
  }

  public void setManual(double speed) {
    
    System.out.println("encoder: "+elevatorEncoder.getRaw());
    elevatorMotor1.set(speed);
    elevatorMotor2.set(-speed);
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
  public void periodic() {
    
  }
}
