// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.drive.FollowPathCommand;
import frc.robot.commands.auto.drive.StopCommand;
import frc.robot.commands.auto.positionable.AutoAimShooterCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePiecePath extends SequentialCommandGroup {
  /** Creates a new OnePiecePath. */
  public OnePiecePath(DriveSubsystem drive, ShooterSubsystem shooter, LimelightSubsystem shooterLimelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PiratePath path = new PiratePath("OnePiecePath", false);
    var paths = path.getSubPaths();
    var move = paths.get(0);
    
    addCommands(
      new InstantCommand(() -> {
        drive.setDefensiveMode(true);
      }, drive),
      
      

        new AutoAimShooterCommand(shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),
      
        new WaitCommand(2),
        new InstantCommand(()->{
          shooter.setFeeder(1);
        }),

        new WaitCommand(0.3),

        new InstantCommand(()->{
          shooter.setShooter(0);
          shooter.setFeeder(0);
        }, shooter),

        new FollowPathCommand(drive, move, true, 0),
        new StopCommand(drive)
        
    );
  }
}
