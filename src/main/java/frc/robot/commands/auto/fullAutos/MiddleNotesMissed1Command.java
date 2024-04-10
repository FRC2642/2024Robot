// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.IntakeUntilFound;
import frc.robot.commands.auto.drive.DivertToGamePieceCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.commands.auto.drive.FollowPathCommand;
import frc.robot.commands.auto.positionable.AutoAimShooterCommand;
import frc.robot.commands.auto.positionable.SetIntakeCommand;
import frc.robot.path.PiratePath;
import frc.robot.utils.VectorR;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleNotesMissed1Command extends SequentialCommandGroup {
  /** Creates a new MiddleNotesMissed1Command. */
  public MiddleNotesMissed1Command(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, LimelightSubsystem shooterLimelight, LimelightSubsystem intakeLimelight) {
    PiratePath path = new PiratePath("MiddleNotePathMissed1", false);
    var paths = path.getSubPaths();
    var getNote2 = paths.get(0);
    var shootNote2 = paths.get(1);
    var getNote3 = paths.get(2);
    var shootNote3 = paths.get(3);
    
    addCommands(
      new InstantCommand(() -> {
        drive.setDefensiveMode(true);
        shooter.setSpeedLimit(0.9);
        intake.setSpeedLimit(0.5);
        shooter.setShooter(-1);
        drive.move(new VectorR(), 0);
        DriveSubsystem.resetDisplacement(new VectorR());
        DriveSubsystem.resetGyro(0);
        
      }, drive, intake, shooter),

      new DivertToGamePieceCommand(drive, intakeLimelight, DetectionType.NOTE, getNote2, true, 0, 0.3, 0.2, true).alongWith(
        new IntakeUntilFound(()->IntakePosition.EXTENDED, intake, shooter, false)
      ),

      new FollowPathCommand(drive, shootNote2, false, 0).alongWith(
        new SetIntakeCommand(intake, ()->IntakePosition.RETRACTED)
      ),

      new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),
      new WaitCommand(0.5),
      new InstantCommand(()->shooter.setFeeder(1), shooter),
      new WaitCommand(0.3),
      new InstantCommand(()->{
        shooter.setShooter(0);
        shooter.setFeeder(0);
      }, shooter),

      new DivertToGamePieceCommand(drive, intakeLimelight, DetectionType.NOTE, getNote3, true, 0, 0.3, 0.2, true).alongWith(
        new IntakeUntilFound(()->IntakePosition.EXTENDED, intake, shooter, false)
      ),

      new FollowPathCommand(drive, shootNote3, false, 0).alongWith(
        new SetIntakeCommand(intake, ()->IntakePosition.RETRACTED)
      ),

      new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),
      new WaitCommand(0.5),
      new InstantCommand(()->shooter.setFeeder(1), shooter),
      new WaitCommand(0.3),
      new InstantCommand(()->{
        shooter.setShooter(0);
        shooter.setFeeder(0);
      }, shooter)
    );
  }
}
