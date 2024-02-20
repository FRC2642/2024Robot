// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.drive.AutoLockOntoSpeakerCommand;
import frc.robot.commands.auto.drive.DivertToGamePieceCommand;
import frc.robot.commands.auto.drive.FollowPathCommand;
import frc.robot.commands.auto.positionable.SetElevatorCommand;
import frc.robot.commands.auto.positionable.SetIntakeCommand;
import frc.robot.commands.auto.positionable.SetShooterCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.subsystems.ShooterSubsystem.ShooterPosition;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.utils.Easings.Functions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargeAmpN2Pc extends SequentialCommandGroup {
  /** Creates a new ChargeAmpN2Pc. */
  public ChargeAmpN2Pc(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, LimelightSubsystem limelight, ElevatorSubsystem elevator) {
    
    ArrayList<Double> times = new ArrayList<>();
    times.add(0.0); times.add(1.1); times.add(2.2); times.add(3.3); times.add(4.4); times.add(5.5); times.add(6.6); times.add(7.7); times.add(8.8);
    PiratePath path = new PiratePath("2 Pc Charged Amp", false);
    path.fillWithSubPointsEasing(0.01, Functions.easeLinear);
    var paths = path.getSubPaths(times, 0.01);
    var driveToShoot1 = paths.get(0);
    var driveToIntake1 = paths.get(1);
    var driveToAmp1 = paths.get(2);
    var driveToIntake2 = paths.get(3);
    var driveToAmp2 = paths.get(4);
    var driveToIntake3 = paths.get(5);
    var driveToShoot2 = paths.get(6);
    var driveOut = paths.get(7);


    addCommands(
      //******Drive To Shoot 1st Note and Drop and Run Intake******//

      //Drive to Shooting Spot And Rev Shooter
      new AutoLockOntoSpeakerCommand(drive, limelight, elevator, shooter, DetectionType.FIDUCIAL, driveToShoot1, true, 0.25).alongWith(
        new InstantCommand(()->{
          shooter.setShooter(0.7);
        }, shooter)
      ),

      //Shoot note and drop intake and run intake
      new InstantCommand(()->{
        shooter.setFeeder(0.5);
        intake.setIntake(0.5);
      }, shooter, intake).alongWith(
        new SetIntakeCommand(intake, ()->IntakePosition.EXTENDED),
        
        //Stop shooter after 0.5 seconds
        new WaitCommand(0.5).andThen(
          new InstantCommand(()->{
            shooter.setFeeder(0);
            shooter.setShooter(0);
        }, shooter)
        ),        

        //******Intake 2nd Note******//

        //Intake next note as note is being shot
        new DivertToGamePieceCommand(drive, limelight, DetectionType.NOTE, driveToIntake1, false, 0.25, 0.3, 0.5, true)
      ),

      //Stop Intake
      new InstantCommand(()->{
        intake.setIntake(0);
      }, intake),

      //******Drive And Deposit 2nd Note Into Amp Then Intake 3rd Note******//

      //Drive To Amp And Prep For Shoot
      new FollowPathCommand(drive, driveToAmp1, false, 0.25).alongWith(
          new SetElevatorCommand(elevator, ()->ElevatorPosition.AMP),
          new SetShooterCommand(shooter, ()->ShooterPosition.AMP),
          new InstantCommand(()->{
            shooter.setShooter(0.3);
          }, shooter)
      ),

      //Shoot note and drop intake and run intake
      new InstantCommand(()->{
        shooter.setFeeder(0.5);
        intake.setIntake(0.5);
      }, shooter, intake).alongWith(
        new SetIntakeCommand(intake, ()->IntakePosition.EXTENDED),
        
        //Stop shooter after 0.5 seconds
        new WaitCommand(0.5).andThen(
          new InstantCommand(()->{
            shooter.setFeeder(0);
            shooter.setShooter(0);
          }, shooter)
        ),        

        //Intake next note as note is being shot
        new DivertToGamePieceCommand(drive, limelight, DetectionType.NOTE, driveToIntake2, false, 0.25, 0.3, 0.5, true)
      ),

      //Stop intake
      new InstantCommand(()->{
        intake.setIntake(0);
      }, intake),

      //******Drive And Deposit 3rd Note Into Amp Then Intake 4th Note******//

      //Drive To Amp And Prep For Shoot
      new FollowPathCommand(drive, driveToAmp2, false, 0.25).alongWith(
        new SetElevatorCommand(elevator, ()->ElevatorPosition.AMP),
        new SetShooterCommand(shooter, ()->ShooterPosition.AMP),
        new InstantCommand(()->{
          shooter.setShooter(0.7);
        }, shooter)
      ),

      //Shoot note and drop intake and run intake
      new InstantCommand(()->{
        shooter.setFeeder(0.5);
        intake.setIntake(0.5);
      }, shooter, intake).alongWith(
        new SetIntakeCommand(intake, ()->IntakePosition.EXTENDED),
        
        //Stop shooter after 0.5 seconds
        new WaitCommand(0.5).andThen(
          new InstantCommand(()->{
            shooter.setFeeder(0);
            shooter.setShooter(0);
          }, shooter)
        ),        

        //Intake next note as note is being shot
        new DivertToGamePieceCommand(drive, limelight, DetectionType.NOTE, driveToIntake3, false, 0.25, 0.3, 0.5, true)
      ),

      //Stop intake
      new InstantCommand(()->{
        intake.setIntake(0);
      }, intake),

      //******Drive To Shoot 4th Note and Drop and Run Intake******//

      //Drive to next position and lock onto speaker and rev shooter
      new AutoLockOntoSpeakerCommand(drive, limelight, elevator, shooter, DetectionType.FIDUCIAL, driveToShoot2, true, 0.25).alongWith(
        new InstantCommand(()->{
          shooter.setShooter(0.7);
        }, shooter)
      ),

      //Shoot note and start intake
      new InstantCommand(()->{
        shooter.setFeeder(0.5);
        intake.setIntake(0.5);
      }, shooter, intake).alongWith(
        //Stop shooter after 0.5 seconds
        new WaitCommand(0.5).andThen(
          new InstantCommand(()->{
            shooter.setFeeder(0);
            shooter.setShooter(0);
          }, shooter)
        ),
        
        //Drive out to last note as note is being shot
        new DivertToGamePieceCommand(drive, limelight, DetectionType.NOTE, driveOut, false, 0.25, 0.3, 0.5, true)
      )
    );
  }
}
