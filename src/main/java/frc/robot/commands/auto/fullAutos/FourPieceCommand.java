// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.IntakeUntilFound;
import frc.robot.commands.auto.drive.DivertToGamePieceCommand;
import frc.robot.commands.auto.drive.FollowPathCommand;
import frc.robot.commands.auto.drive.LimelightPathCommand;
import frc.robot.commands.auto.drive.StopCommand;
import frc.robot.commands.auto.positionable.AutoAimShooterCommand;
import frc.robot.commands.auto.positionable.AutoAngleShooterCommand;
import frc.robot.commands.auto.positionable.SetIntakeCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.subsystems.RobotState.RobotConfiguration;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;
import frc.robot.utils.VectorR;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourPieceCommand extends SequentialCommandGroup {
  /** Creates a new FrontTwoPiece. */
  public FourPieceCommand(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, LimelightSubsystem shooterLimelight, LimelightSubsystem intakeLimelight) {
    PiratePath path = new PiratePath("FourPiecePath", false);
    var paths = path.getSubPaths();
    var shootNote = paths.get(0);
    var getNoteShootNote = paths.get(1);
    var get3rdNote = paths.get(2);
    var get4thNote = paths.get(3);
    var shoot4thNote = paths.get(4);

    shooterLimelight.setDetectionType(DetectionType.FIDUCIAL);
    intakeLimelight.setDetectionType(DetectionType.NOTE);

    addCommands(
      new InstantCommand(() -> {
        drive.setDefensiveMode(true);
        intake.setSpeedLimit(0.65);
        shooter.setSpeedLimit(0.4);
        drive.move(new VectorR(), 0);
        shooter.setShooter(0.8);
      }, drive, intake, shooter),

        //1ST NOTE
        new FollowPathCommand(drive, shootNote, true, 0).alongWith(
          new SetIntakeCommand(intake, ()->IntakePosition.EXTENDED)
          
        ),
        new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),
      
        
        new InstantCommand(()->{
          shooter.setFeeder(1);
          intake.setIntake(1);
          drive.move(new VectorR(), 0);
        }, shooter, drive),

        new WaitCommand(0.3),

        //2ND NOTE
        new LimelightPathCommand(drive, getNoteShootNote, false, 0.25, shooterLimelight).deadlineWith(
          new RunCommand(()->{
            shooter.tiltToAngle(shooter.getAutoAngle(-12));
          })
        ),



        //3RD NOTE
        new InstantCommand(()->{
          shooter.setShooter(0);
          shooter.setFeeder(0);
          shooter.setSpeedLimit(0.4);
        }, shooter),

        new DivertToGamePieceCommand(drive, intakeLimelight, DetectionType.NOTE, get3rdNote, false, 0.25, 0.2, 1.4, true).alongWith(
          new IntakeUntilFound(()->IntakePosition.EXTENDED, intake, shooter, false)
        ).withTimeout(3),

        new InstantCommand(()->{
          intake.setIntake(0);
          shooter.setFeeder(-0.2);
          drive.move(new VectorR(), 0);
        }, intake, shooter, drive),

        new WaitCommand(0.1),
        new InstantCommand(()->{
          shooter.setFeeder(0);
        }, shooter),

        new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),

        new InstantCommand(()->{
            shooter.setFeeder(1);
          }, shooter),
        

        new WaitCommand(0.3),
        new InstantCommand(()->{
          shooter.setShooter(0);
          shooter.setFeeder(0);
          shooter.stopShooter();
        }),

        //4TH NOTE
        new DivertToGamePieceCommand(drive, intakeLimelight, DetectionType.NOTE, get4thNote, false, 0.25, 0.2, 3, true).alongWith(
          new IntakeUntilFound(()->IntakePosition.EXTENDED, intake, shooter, false)
        ).withTimeout(5),

        
        new InstantCommand(()->{
          intake.setIntake(0);
          shooter.setFeeder(-0.2);
          drive.move(new VectorR(), 0);
        }, intake, shooter, drive),

        new WaitCommand(0.2),
        new InstantCommand(()->{
          shooter.setFeeder(0);
        }, shooter),

        new FollowPathCommand(drive, shoot4thNote, false, 0.25).alongWith(
          new SetIntakeCommand(intake, ()->IntakePosition.RETRACTED),
          new InstantCommand(()->{
            shooter.setShooter(0.8);
          })
        ),
        new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),

        new WaitCommand(0.1),

        new InstantCommand(()->{
          shooter.setFeeder(1);
        }, shooter),

        new WaitCommand(0.4),
        new InstantCommand(()->{
          shooter.setShooter(0);
          shooter.setFeeder(0);
          shooter.setManual(0);
        }),

        new StopCommand(drive)
    );
  }
}