// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.IntakeUntilFound;
import frc.robot.commands.auto.drive.FollowPathCommand;
import frc.robot.commands.auto.drive.StopCommand;
import frc.robot.commands.auto.positionable.AutoAimShooterCommand;
import frc.robot.commands.auto.positionable.SetIntakeCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;
import frc.robot.utils.VectorR;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OptimizedThreePiece extends SequentialCommandGroup {
  /** Creates a new FrontTwoPiece. */
  public OptimizedThreePiece(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, LimelightSubsystem shooterLimelight) {
    PiratePath path = new PiratePath("OptimizedThreePiece", false);
    var paths = path.getSubPaths();
    var shootNote = paths.get(0);
    var getNoteShootNote = paths.get(1);
    var getNote = paths.get(2);
    var shootNote2 = paths.get(3);

    addCommands(
      new InstantCommand(() -> {
        drive.setDefensiveMode(true);
        intake.setSpeedLimit(0.65);
        shooter.setSpeedLimit(0.5);
        DriveSubsystem.resetGyro(0);
      }, drive, intake, shooter),

        //1ST NOTE
        new FollowPathCommand(drive, shootNote, true, 0.25).alongWith(
          new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),
          new SetIntakeCommand(intake, ()->IntakePosition.EXTENDED)
        ),
      
        new InstantCommand(()->{
          shooter.setFeeder(1);
          drive.move(new VectorR(), 0);
        }, shooter, drive),

        new WaitCommand(0.1),

        //2ND NOTE
        new FollowPathCommand(drive, getNoteShootNote, false, 0.25).alongWith(
          new IntakeUntilFound(()->IntakePosition.EXTENDED, intake, shooter)
        ).withTimeout(3),

        
        new InstantCommand(()->{
          intake.setIntake(0);
          shooter.setFeeder(-0.2);
          drive.move(new VectorR(), 0);
        }, intake, shooter, drive),

        new WaitCommand(0.2),
        new InstantCommand(()->{
          shooter.setFeeder(0);
        }, shooter),

        new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),
        

        new WaitCommand(0.5),
        new InstantCommand(()->{
          shooter.setFeeder(1);
        }, shooter),

        new WaitCommand(0.1),

        //3RD NOTE
        new InstantCommand(()->{
          shooter.setShooter(0);
          shooter.setFeeder(0);
        }, shooter),

        new FollowPathCommand(drive, getNote, false, 0.25).alongWith(
          new IntakeUntilFound(()->IntakePosition.EXTENDED, intake, shooter)
        ).withTimeout(3),

        new InstantCommand(()->{
          intake.setIntake(0);
          shooter.setFeeder(-0.2);
        }, intake, shooter, drive),

        new WaitCommand(0.1).alongWith(
          new FollowPathCommand(drive, shootNote2, false, 0.25)
        ).andThen(
          new InstantCommand(()->{
            shooter.setFeeder(0);
            drive.move(new VectorR(), 0);
          }, shooter)
        ),
        
        new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),

        new InstantCommand(()->{
          shooter.setFeeder(1);
        }, shooter),

        new WaitCommand(0.4),
        new InstantCommand(()->{
          shooter.setShooter(0);
          shooter.setFeeder(0);
        }),

        new StopCommand(drive)
    );
  }
}
