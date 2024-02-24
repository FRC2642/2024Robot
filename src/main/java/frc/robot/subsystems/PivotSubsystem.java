// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */

  private CANSparkMax pivotMotor = new CANSparkMax(0, MotorType.kBrushless); // CHANGE DEVICE IDs
  private CANcoder encoder = new CANcoder(0);

  public void setPivotSpeed(float pivotSpeed) {
    pivotMotor.set(pivotSpeed);
  }

  public PivotSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
