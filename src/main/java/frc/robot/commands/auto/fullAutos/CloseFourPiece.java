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
import frc.robot.commands.auto.positionable.AutoAimShooterCommand;
import frc.robot.commands.auto.positionable.AutoAngleShooterCommand;
import frc.robot.commands.auto.positionable.SetIntakeCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;
import frc.robot.utils.VectorR;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CloseFourPiece extends SequentialCommandGroup {
  /** Creates a new CloseFourPiece. */
  public CloseFourPiece(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, LimelightSubsystem shooterLimelight, LimelightSubsystem intakeLimelight) {
    PiratePath path = new PiratePath("CloseFourPiece", false);
    var paths = path.getSubPaths();
    var note2 = paths.get(0);
    var shootNote2 = paths.get(1);
    var note3 = paths.get(2);
    var note4 = paths.get(3);
    var shootNote4 = paths.get(4);
    var moveOut = paths.get(5);

    
    
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

      //Get 2nd note
      new DivertToGamePieceCommand(drive, intakeLimelight, DetectionType.NOTE, note2, true, 0.25, 0.2, 0.8, true).alongWith(
          new IntakeUntilFound(()->IntakePosition.EXTENDED, intake, shooter)
      ).withTimeout(3),

      new FollowPathCommand(drive, shootNote2, false, 0.25),

      //Shoot 2nd note
      new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),
      new WaitCommand(0.5),
      new InstantCommand(()->shooter.setFeeder(1), shooter),
      new WaitCommand(0.3),
      new InstantCommand(()->{
        shooter.setShooter(0);
        shooter.setFeeder(0);
      }, shooter),

      //Get 3rd note
      new DivertToGamePieceCommand(drive, intakeLimelight, DetectionType.NOTE, note3, false, 0.25, 0.2, 1, true).alongWith(
          new IntakeUntilFound(()->IntakePosition.EXTENDED, intake, shooter)
      ).withTimeout(3),

      //Shoot 3rd note
      new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),
      new WaitCommand(0.5),
      new InstantCommand(()->shooter.setFeeder(1), shooter),
      new WaitCommand(0.3),
      new InstantCommand(()->{
        shooter.setShooter(0);
        shooter.setFeeder(0);
      }, shooter),

      //Get 4th note
      new DivertToGamePieceCommand(drive, intakeLimelight, DetectionType.NOTE, note4, false, 0.25, 0.2, 1, true).alongWith(
          new IntakeUntilFound(()->IntakePosition.EXTENDED, intake, shooter)
      ).withTimeout(3),

      new FollowPathCommand(drive, shootNote4, false, 0.25).alongWith(new SetIntakeCommand(intake, ()->IntakePosition.RETRACTED)),

      //Shoot 4th note
      new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),
      new WaitCommand(0.5),
      new InstantCommand(()->shooter.setFeeder(1), shooter),
      new WaitCommand(0.3),
      new InstantCommand(()->{
        shooter.setShooter(0);
        shooter.setFeeder(0);
        shooter.setManual(0);
      }, shooter),

      new FollowPathCommand(drive, moveOut, false, 0.25)

    );
  }
}
