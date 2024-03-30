// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.positionable;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.RobotState.RobotConfiguration;

public class SetRobotConfigurationCommand extends ParallelCommandGroup {
  private final RobotConfiguration configuration;

  public SetRobotConfigurationCommand(RobotConfiguration config, ShooterSubsystem shooter, IntakeSubsystem intake) {
    configuration = config;
   
    addCommands(
      new SetShooterCommand(shooter, () -> configuration.shooterAngle),
      new SetIntakeCommand(intake, () -> configuration.intakePos)
    );
    
  }

  

}
