// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.drive.FollowPathCommand;
import frc.robot.commands.auto.positionable.SetIntakeCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Amp3Pc extends SequentialCommandGroup {
  /** Creates a new Amp2Pc. */
  public Amp3Pc(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, LimelightSubsystem limelight) {
    PiratePath path = new PiratePath("Basic 3Pc Amp", false);
    var paths = path.getSubPaths();

    var driveToShoot1 = paths.get(0);
    var driveToIntake1 = paths.get(1);
    var driveToShoot2 = paths.get(2);
    var driveToIntake2 = paths.get(3);
    var driveToShoot3 = paths.get(4);

    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new FollowPathCommand(drive, driveToShoot1, true, 0.25).alongWith(
        new InstantCommand(()->{
          shooter.tiltToAngle(50);
          shooter.setShooterRPM();
        }, shooter)
      ), 

      new InstantCommand(()->{
        shooter.setFeeder(0.5);
        intake.setIntake(0.5);
      }, shooter).alongWith(

        new SetIntakeCommand(intake, ()->IntakePosition.EXTENDED),

        new WaitCommand(.5).andThen(
          new InstantCommand(()->{
            shooter.setFeeder(0);
            shooter.setShooter(0);
            shooter.tiltToAngle(0);
          }, shooter)
        ),

        new FollowPathCommand(drive, driveToIntake1, false, 0.25)
      ),

      new WaitCommand(1).andThen(
        new InstantCommand(()->{
          intake.setIntake(0);
        }, intake).alongWith(

          new SetIntakeCommand(intake, ()->IntakePosition.RETRACTED)
        )
      ),

      new FollowPathCommand(drive, driveToShoot2, false, 0.25).alongWith(
        new InstantCommand(()->{
          shooter.tiltToAngle(22);
          shooter.setShooterRPM();
        }, shooter)
      ),

      new InstantCommand(()->{
        shooter.setFeeder(0.5);
        intake.setIntake(0.5);
      }, shooter).alongWith(

        new WaitCommand(0.5).andThen(
          new InstantCommand(()->{
            shooter.setFeeder(0);
            shooter.setShooter(0);
            shooter.tiltToAngle(0);
          },shooter)
        ),

        new SetIntakeCommand(intake, ()->IntakePosition.EXTENDED),

        new FollowPathCommand(drive, driveToIntake2, false, 0.25)
      ),

      new WaitCommand(1).andThen(
        new InstantCommand(()->{
          intake.setIntake(0);
        }, intake).alongWith(

          new SetIntakeCommand(intake, ()->IntakePosition.RETRACTED)
        )
      ),

      new FollowPathCommand(drive, driveToShoot3, false, 0.25).alongWith(
        new InstantCommand(()->{
          shooter.tiltToAngle(22);
          shooter.setShooterRPM();
        }, shooter)
      ),

      new InstantCommand(()->{
        shooter.setFeeder(0.5);
      }, shooter).alongWith(

        new WaitCommand(0.5).andThen(
          new InstantCommand(()->{
            shooter.setFeeder(0);
            shooter.setShooter(0);
            shooter.tiltToAngle(0);
          },shooter)
        )
      )
    );
  }
}
