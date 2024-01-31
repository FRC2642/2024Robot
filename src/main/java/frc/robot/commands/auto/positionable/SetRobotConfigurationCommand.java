// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.positionable;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.ShooterSubsystem.ShooterPosition;

public class SetRobotConfigurationCommand extends ParallelCommandGroup {
  private final RobotConfiguration configuration;

  public SetRobotConfigurationCommand(RobotConfiguration config, ShooterSubsystem shooter, ElevatorSubsystem elevator, IntakeSubsystem intake) {
    configuration = config;
   
    addCommands(
      new SetShooterCommand(shooter, () -> configuration.shooterPos),
      new SetElevatorCommand(elevator, () -> configuration.elevatorPos),
      new SetIntakeCommand(intake, () -> configuration.intakePos)
    );
    
  }

  public enum RobotConfiguration {
    SHOOT_SPEAKER(ElevatorPosition.TRAVEL, IntakePosition.RETRACTED, ShooterPosition.MANUAL),
    SHOOT_AMP(ElevatorPosition.AMP, IntakePosition.RETRACTED, ShooterPosition.AMP),
    SHOOT_TRAP(ElevatorPosition.TRAP, IntakePosition.RETRACTED, ShooterPosition.TRAP),
    INTAKE(ElevatorPosition.TRAVEL, IntakePosition.EXTENDED, ShooterPosition.TRAVEL),
    TRAVEL(ElevatorPosition.TRAVEL, IntakePosition.RETRACTED, ShooterPosition.TRAVEL);

    public final ShooterPosition shooterPos;
    public final ElevatorPosition elevatorPos;
    public final IntakePosition intakePos;
    
    private RobotConfiguration( 
      ElevatorPosition elevatorPos,
      IntakePosition intakePos,
      ShooterPosition shooterPos) {
      this.shooterPos = shooterPos;
      this.elevatorPos = elevatorPos;
      this.intakePos = intakePos;
    }

  }
}
