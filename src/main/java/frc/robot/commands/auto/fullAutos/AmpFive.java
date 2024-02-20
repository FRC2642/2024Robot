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
import frc.robot.commands.auto.positionable.SetIntakeCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.utils.Easings.Functions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpFive extends SequentialCommandGroup {
  /** Creates a new AmpFive. */
  public AmpFive(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, LimelightSubsystem limelight, ElevatorSubsystem elevator) {
    ArrayList<Double> times = new ArrayList<Double>();
    times.add(0.0); times.add(1.107); times.add(1.771); times.add(2.609); times.add(4.354); times.add(5.939); times.add(7.405); times.add(8.834); times.add(10.728); times.add(12.927); times.add(15.332);
    PiratePath path = new PiratePath("5 Pc HP", false);
    path.fillWithSubPointsEasing(0.01, Functions.easeLinear);
    var paths = path.getSubPaths(times, 0.01);
    var driveToShoot1 = paths.get(0);
    var driveToIntake1 = paths.get(1);
    var driveToShoot2 = paths.get(3);
    var driveToIntake2 = paths.get(4);
    var driveToShoot3 = paths.get(5);
    var driveToIntake3 = paths.get(6);
    var driveToShoot4 = paths.get(7);
    var driveToIntake4 = paths.get(8);
    var driveToShoot5 = paths.get(9);
    var driveOut = paths.get(10);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //******1ST AND 2ND NOTE*****//
      //Drive to position and lock onto speaker and rev shooter
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
          
        //Intake next note as note is being shot
        new DivertToGamePieceCommand(drive, limelight, DetectionType.NOTE, driveToIntake1, false, 0.25, 0.3, 0.5, true)
      ),

      //Stop intake
      new InstantCommand(()->{
        intake.setIntake(0);
      }, intake),
      

      //*****2ND AND 3RD NOTE*****//
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
          
        //Intake next note as note is being shot
        new DivertToGamePieceCommand(drive, limelight, DetectionType.NOTE, driveToIntake2, false, 0.25, 0.3, 0.5, true)
      ),

      //Stop intake
      new InstantCommand(()->{
        intake.setIntake(0);
      }, intake),

      //*****3RD AND 4TH NOTE*****//
      //Drive to next position and lock onto speaker and rev shooter
      new AutoLockOntoSpeakerCommand(drive, limelight, elevator, shooter, DetectionType.FIDUCIAL, driveToShoot3, false, 0.25).alongWith(
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
        new WaitCommand(.5).andThen(
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


      //*****4TH AND 5TH NOTE*****//
      //Drive to next position and lock onto speaker and rev shooter
      new AutoLockOntoSpeakerCommand(drive, limelight, elevator, shooter, DetectionType.FIDUCIAL, driveToShoot4, false, 0.25).alongWith(
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
        new WaitCommand(.5).andThen(
          new InstantCommand(()->{
            shooter.setFeeder(0);
            shooter.setShooter(0);
          }, shooter)
        ),

        //Intake next note as note is being shot
        new DivertToGamePieceCommand(drive, limelight, DetectionType.NOTE, driveToIntake4, false, 0.25, 0.3, 0.5, true)
      ),
      
      //Stop intake
      new InstantCommand(()->{
        intake.setIntake(0);
      }, intake),

      //Move to next position and lock onto speaker and rev shooter
      new AutoLockOntoSpeakerCommand(drive, limelight, elevator, shooter, DetectionType.FIDUCIAL, driveToShoot5, false, 0.25).alongWith(
        new InstantCommand(()->{
          shooter.setShooter(0.5);
        }, shooter)
      ),

      //Shoot note and start intake
      new InstantCommand(()->{
        shooter.setFeeder(0.5);
      }, shooter).alongWith(
        //Stop shooter after 0.5 seconds
        new WaitCommand(.5).andThen(
          new InstantCommand(()->{
            shooter.setFeeder(0);
            shooter.setShooter(0);
          }, shooter)
        ),
      

        //Drive Out
        new DivertToGamePieceCommand(drive, limelight, DetectionType.NOTE, driveOut, false, 0.25, 0.3, 0.5, true)
      )
    );
  }
}
