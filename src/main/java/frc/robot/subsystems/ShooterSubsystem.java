// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  // Replace deviceIds when recieved
  private CANSparkMax backRoller = new CANSparkMax(0, MotorType.kBrushless);
  private TalonFX flyWheels = new TalonFX(0);

  void runBackRoller(double rollerSpeed) {
    backRoller.set(rollerSpeed);
  }

  void runFlyWheels(double flyWheelSpeed) {
    flyWheels.set(flyWheelSpeed);
  }

  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
