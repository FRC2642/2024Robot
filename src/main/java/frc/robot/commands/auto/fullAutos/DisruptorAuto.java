// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.IntakeUntilFound;
import frc.robot.commands.auto.drive.DivertToGamePieceCommand;
import frc.robot.commands.auto.drive.FollowPathCommand;
import frc.robot.commands.auto.drive.StopCommand;
import frc.robot.commands.auto.positionable.AutoAimShooterCommand;
import frc.robot.commands.auto.positionable.AutoAngleShooterCommand;
import frc.robot.commands.auto.positionable.SetShooterCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;
import frc.robot.utils.VectorR;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DisruptorAuto extends SequentialCommandGroup {
  /** Creates a new DisruptorAuto. */
  public DisruptorAuto(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, LimelightSubsystem shooterLimelight, LimelightSubsystem intakeLimelight) {
    PiratePath path = new PiratePath("Disruptor", false);
    
    addCommands(
      new InstantCommand(() -> {
        drive.setDefensiveMode(true);
        shooter.setSpeedLimit(0.9);
        intake.setSpeedLimit(0.5);
        drive.move(new VectorR(), 0);
        shooter.setShooter(-1);
      }, drive, intake, shooter),

    
      //Shoot 1st note
      new AutoAngleShooterCommand(shooter, shooterLimelight),
      new InstantCommand(()->shooter.setManual(0), shooter),
      new WaitCommand(0.3),
      new InstantCommand(()->shooter.setFeeder(1), shooter),
      new WaitCommand(0.3),
      new InstantCommand(()->{
        shooter.setShooter(0);
        shooter.setFeeder(0);
      }, shooter),

      //Disrupt notes
      new FollowPathCommand(drive, path, true, 0).alongWith(
        new SetShooterCommand(shooter, ()->ShooterAngle.TRAVEL)
      ),

      new StopCommand(drive)
    );
  }
}
