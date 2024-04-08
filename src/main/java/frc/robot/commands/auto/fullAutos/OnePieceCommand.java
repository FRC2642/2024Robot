// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.positionable.AutoAimShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceCommand extends SequentialCommandGroup {
  /** Creates a new OnePieceCommand. */
  public OnePieceCommand(DriveSubsystem drive, ShooterSubsystem shooter, LimelightSubsystem shooterLimelight) {
    
    addCommands(
      new InstantCommand(() -> {
        shooter.setSpeedLimit(0.9);
      }, shooter),

        new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),
      
        new WaitCommand(0.5),
        new InstantCommand(()->{
          shooter.setFeeder(1);
        }, shooter),

        new WaitCommand(0.3),

        new InstantCommand(()->{
          shooter.setShooter(0);
          shooter.setFeeder(0);
        }, shooter)
    );
  }
}
