// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.drive.DriveDistanceCommand;
import frc.robot.commands.auto.drive.FollowPathCommand;
import frc.robot.commands.auto.positionable.SetShooterAngleCommand;
import frc.robot.commands.teleop.resetters.ResetGyroCommand;
import frc.robot.path.PiratePath;
import frc.robot.path.PiratePoint;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;
import frc.robot.utils.VectorR;
import frc.robot.utils.Easings.Functions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceTest extends SequentialCommandGroup {

  public OnePieceTest(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake) {
    PiratePath path = new PiratePath(false);
    path.add(new PiratePoint(0, 0, 298.74, 0, false));
    path.add(new PiratePoint(2, 0, 298.74, 4, true));
    path.fillWithSubPointsEasing(0.05, Functions.easeOutExpo);
    
    addCommands(
      new InstantCommand(() -> {
        drive.setDefensiveMode(true);
      }, drive),
      new ResetGyroCommand(298.74),
      

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
          shooter.set(ShooterSpeed.TRAVEL);
          shooter.setFeeder(0);
        }, shooter)
        
        //new DriveDistanceCommand(drive, VectorR.fromPolar(0.2, 0), 5)
    );
  }
}
