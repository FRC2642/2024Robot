// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.drive.DivertToGamePieceCommand;
import frc.robot.commands.auto.drive.FollowPathCommand;
import frc.robot.commands.auto.positionable.SetIntakeCommand;
import frc.robot.commands.teleop.ManualIntakeCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveCommand extends SequentialCommandGroup {
  /** Creates a new MoveCommadn. */
  
  public MoveCommand(DriveSubsystem drive, LimelightSubsystem intakeLimelight, IntakeSubsystem intake) {
    PiratePath path = new PiratePath("MovePath", false);
    var paths = path.getSubPaths();
    var move = paths.get(0);
    addCommands(
      new SetIntakeCommand(intake, ()->IntakePosition.EXTENDED),
      new DivertToGamePieceCommand(drive, intakeLimelight, DetectionType.NOTE, move, true, 0.25, 0.1, 0.1, true)
      
    );
  }
}
