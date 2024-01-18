// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MathR;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private PIDController tiltPID = new PIDController(0.2, 0, 0);
  private PIDController elevatorPID = new PIDController(0.2, 0, 0);

  private CANSparkMax tiltMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax tiltMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private AbsoluteEncoder tiltEncoder = tiltMotor1.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);

  private CANSparkMax feederMotor = new CANSparkMax(0, MotorType.kBrushless);

  private CANSparkMax elevatorMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax elevatorMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private AbsoluteEncoder elevatorEncoder = tiltMotor1.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);

  
  public ShooterSubsystem() {
    tiltMotor2.follow(tiltMotor1);
    elevatorMotor2.follow(elevatorMotor1);
  }

  public double getPitch(){
    return tiltEncoder.getPosition() * Constants.SHOOTER_TILT_ENCODER_TICKS_PER_DEGREE;
  }

  public double getHeight(){
    return elevatorEncoder.getPosition() * Constants.ELEVATOR_ENCODER_TICKS_PER_FOOT;
  }

  public void tiltToAngle(double degrees){
    tiltMotor1.set(MathR.limit(tiltPID.calculate(getPitch(), degrees), -1, 1));
  }

  public void moveToPosition(double feet){
    elevatorMotor1.set(MathR.limit(elevatorPID.calculate(getHeight(), feet), -1, 1));
  }

  public void moveFeeder(double speed){
    feederMotor.set(speed);
  }

  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
