// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.auto.positionable.SetRobotConfigurationCommand;
import frc.robot.commands.auto.positionable.SetRobotConfigurationCommand.RobotConfiguration;
import frc.robot.commands.teleop.RunShooterCommand;
import frc.robot.commands.teleop.DriveCommands.LockOntoSpeakerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSpeakerCommand extends ParallelDeadlineGroup {
  /** Creates a new ShootSpeakerCommand. */
  public ShootSpeakerCommand(DriveSubsystem drive, ShooterSubsystem shooter, ElevatorSubsystem elevator, IntakeSubsystem intake, LimelightSubsystem limelight, XboxController mainControl) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new RunShooterCommand(shooter, 0.9, mainControl));
    addCommands(
      new SetRobotConfigurationCommand(RobotConfiguration.SHOOT_SPEAKER, shooter, elevator, intake),
      new LockOntoSpeakerCommand(drive, shooter, elevator, limelight, DetectionType.FIDUCIAL, mainControl)
      );
  }
}
