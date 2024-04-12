// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.IntakeUntilFound;
import frc.robot.commands.auto.drive.DivertToGamePieceCommand;
import frc.robot.commands.auto.drive.DriveTowardsGamePieceCommand;
import frc.robot.commands.auto.drive.FollowPathCommand;
import frc.robot.commands.auto.drive.LockOntoSpeakerAndIntakeCommand;
import frc.robot.commands.auto.positionable.AutoAimShooterCommand;
import frc.robot.commands.auto.positionable.AutoAngleShooterCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;
import frc.robot.utils.VectorR;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SixPiece extends SequentialCommandGroup {
  /*

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SixPiece extends SequentialCommandGroup {
  /** Creates a new SixPieceCommand. */
  public SixPiece(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, LimelightSubsystem shooterLimelight, LimelightSubsystem intakeLimelight) {
    PiratePath path = new PiratePath("CloseFivePiece", false);
    var paths = path.getSubPaths();
    ArrayList<Double> times = new ArrayList<>();
    times.add(1.001211621966981);
    var timePaths = path.getSubPaths(times, 0.1);
    
    var note2 = timePaths.get(0);
    
    var note3 = paths.get(0);
    var shootNote3 = paths.get(1);
    var note4 = paths.get(2);
    var shootNote4 = paths.get(3);
    var note5 = paths.get(4);
    var note6 = paths.get(5);
    
    
    addCommands(
      new InstantCommand(() -> {
        drive.setDefensiveMode(true);
        shooter.setSpeedLimit(0.9);
        intake.setSpeedLimit(0.65);
        drive.move(new VectorR(), 0);
        shooter.setShooter(-1);
      }, drive, intake, shooter),

    
      //Shoot 1st note
      new AutoAngleShooterCommand(shooter, shooterLimelight).withTimeout(0.8),
      new InstantCommand(()->shooter.setManual(0), shooter),
      new InstantCommand(()->shooter.setFeeder(1), shooter),
      new WaitCommand(0.05), //DO NOT REMOVE
      new InstantCommand(()->{
        shooter.setFeeder(0);
      }, shooter),

      //Get & Shoot 2nd note
      new LockOntoSpeakerAndIntakeCommand(drive, intakeLimelight, shooterLimelight, DetectionType.NOTE, note2, true, 0.25, 0.2, 0.75, false, shooter, intake),

      //Get 3rd note
      //Needs work
      new FollowPathCommand(drive, shootNote3, false, 0.5),

      //Shoot 3rd note
      new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight).withTimeout(0.5),
      new InstantCommand(()->shooter.setFeeder(1), shooter),
      new WaitCommand(0.05), //DO NOT REMOVE
      new InstantCommand(()->{
        shooter.setFeeder(0);
      }, shooter),

      //Get 4th note
      new DivertToGamePieceCommand(drive, intakeLimelight, DetectionType.NOTE, note4, false, 0.25, 0.35, 0.5, true).alongWith(
          new IntakeUntilFound(()->IntakePosition.EXTENDED, intake, shooter, true)
      ).withTimeout(2),
      

      new FollowPathCommand(drive, shootNote4, false, 0.25).withTimeout(0.8),
      
      //Shoot 4th note
      new WaitCommand(0.4),
      new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),
      new InstantCommand(()->shooter.setFeeder(1), shooter),
      new WaitCommand(0.05), //DO NOT REMOVE
      new InstantCommand(()->{
        shooter.setFeeder(0);
        shooter.setManual(0);
      }, shooter),

      //Get 5th note
      new DivertToGamePieceCommand(drive, intakeLimelight, DetectionType.NOTE, note5, false, 0.25, 0.3, 0, true).alongWith(
          new IntakeUntilFound(()->IntakePosition.EXTENDED, intake, shooter, true)
      ).withTimeout(4),

      //Shoot 5th note
      new WaitCommand(0.4),
      new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),
      new InstantCommand(()->shooter.setFeeder(1), shooter),
      new WaitCommand(0.05), //DO NOT REMOVE
      new InstantCommand(()->{
        shooter.setFeeder(0);
        shooter.setManual(0);
      }, shooter),

      //Get 6th note
      new DivertToGamePieceCommand(drive, intakeLimelight, DetectionType.NOTE, note6, false, 0.25, 0.3, 0, true).alongWith(
          new IntakeUntilFound(()->IntakePosition.EXTENDED, intake, shooter, true)
      ).withTimeout(4),

      //Shoot 6th note
      new WaitCommand(0.1),
      new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),
      new InstantCommand(()->shooter.setFeeder(1), shooter),
      new WaitCommand(0.2),
      new InstantCommand(()->{
        shooter.setShooter(0);
        shooter.setFeeder(0);
        shooter.setManual(0);
      }, shooter)

      
    );
  }
}
