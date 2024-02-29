// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.drive.FollowPathCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Amp1Pc extends SequentialCommandGroup {
  /** Creates a new Amp1Pc. */
  public Amp1Pc(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, LimelightSubsystem limelight) {

    PiratePath path = new PiratePath("Basic 1Pc Amp", false);
    var paths = path.getSubPaths();

    var driveToShoot1 = paths.get(0);
    var driveOut = paths.get(1);

    // Add your commands in the addCommands() call, e.g.
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
      }, shooter).alongWith(

        new WaitCommand(.5).andThen(
          new InstantCommand(()->{
            shooter.setFeeder(0);
            shooter.setShooter(0);
            shooter.tiltToAngle(0);
          }, shooter)
        ),

        new FollowPathCommand(drive, driveOut, false, 0.25)
      )
    );
  }
}
