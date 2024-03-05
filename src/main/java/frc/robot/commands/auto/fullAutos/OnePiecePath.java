// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.drive.DriveDistanceCommand;
import frc.robot.commands.auto.drive.FollowPathCommand;
import frc.robot.commands.auto.drive.StopCommand;
import frc.robot.commands.auto.positionable.SetShooterAngleCommand;
import frc.robot.commands.teleop.resetters.ResetGyroCommand;
import frc.robot.path.PiratePath;
import frc.robot.path.PiratePoint;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;
import frc.robot.utils.Easings.Functions;
import frc.robot.utils.VectorR;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePiecePath extends SequentialCommandGroup {
  /** Creates a new OnePiecePath. */
  public OnePiecePath(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PiratePath path = new PiratePath("OnePiecePath", false);
    var paths = path.getSubPaths();
    var move = paths.get(0);
    
    addCommands(
      new InstantCommand(() -> {
        drive.setDefensiveMode(true);
      }, drive),
      new ResetGyroCommand(0),
      

        new InstantCommand(()->{
          shooter.set(ShooterSpeed.SPEAKER);
        }, shooter),

        new SetShooterAngleCommand(shooter, ()->ShooterAngle.SUBWOOFER),
      
        new WaitCommand(2),
        new InstantCommand(()->{
          shooter.setFeeder(1);
        }),

        new WaitCommand(0.3),

        new InstantCommand(()->{
          shooter.setShooter(0);
          shooter.setFeeder(0);
        }, shooter),

        new FollowPathCommand(drive, move, true, 0.25),
        new StopCommand(drive)
        
    );
  }
}
