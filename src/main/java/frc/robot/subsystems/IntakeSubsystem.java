// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  public CANSparkMax intakeMotor = new CANSparkMax(0,MotorType.kBrushless);
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Intake(){
  intakeMotor.set(0.5);
  }

  public void stop(){
    intakeMotor.set(0);
  }


}
