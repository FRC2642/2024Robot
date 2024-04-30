// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class LockOntoSpeakerAndIntakeCommand extends Command{
  /** Creates a new DivertToGamePieceCommand. */
  DriveSubsystem drive;
  LimelightSubsystem intakeLimelight;
  LimelightSubsystem shooterLimelight;
  LimelightSubsystem.DetectionType object;
  ShooterSubsystem shooter;
  IntakeSubsystem intake;
  double visionSpeed;
  Timer visionTimer = new Timer();
  Timer intakeTimer = new Timer();
  boolean endWhenIntaken;
  boolean shotNote = false;

  
  final double LIMELIGHT_TURN_KP = 0.007;
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.8, 0.00105, 0);
  
  

  public LockOntoSpeakerAndIntakeCommand(DriveSubsystem drive, LimelightSubsystem intakeLimelight, LimelightSubsystem shooterLimelight, LimelightSubsystem.DetectionType object, double visionSpeed,  boolean endWhenIntaken, ShooterSubsystem shooter, IntakeSubsystem intake) {
    this.drive = drive;
    this.endWhenIntaken = endWhenIntaken;
    this.intakeLimelight = intakeLimelight;
    this.shooterLimelight = shooterLimelight;
    this.object = object;
    this.visionSpeed = visionSpeed;
    this.shooter = shooter;
    this.intake = intake;
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeLimelight.setDetectionType(object);
    visionTimer.reset();
    visionTimer.start();
    intakeTimer.reset();
    intakeTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.set(IntakePosition.EXTENDED);
    intake.setIntake(1);
    shooter.setFeeder(1);
    shooter.runVelocity(ShooterSpeed.SPEAKER.rpm, ShooterSpeed.SPEAKER.rpm, ff.calculate(ShooterSpeed.SPEAKER.rpm), ff.calculate(ShooterSpeed.SPEAKER.rpm));

    //Intake
    if (!(ShooterSubsystem.getNoteDetected() || ShooterSubsystem.getCloseNoteDetected())){
      shooter.setFeeder(0.9);
      shooter.tiltToAngle(ShooterAngle.TRAVEL.angle);
    } 
    //Note is inside but not fully
    else if (ShooterSubsystem.getCloseNoteDetected() && !ShooterSubsystem.getNoteDetected()){
      shooter.setFeeder(0.2);
      shooter.tiltToAngle(ShooterAngle.TRAVEL.angle);
    }
    //Note is inside, shooter moving to angle
    else if (ShooterSubsystem.getNoteDetected() && ShooterSubsystem.getCloseNoteDetected() && !shooter.atPitch(shooter.getAutoAngle(shooterLimelight.y, shooterLimelight.a))){
      shooter.setFeeder(0);
      shooter.tiltToAngle(shooter.getAutoAngle(shooterLimelight.y, shooterLimelight.a));
    }
    //Shoot note
    else if (ShooterSubsystem.getNoteDetected() && ShooterSubsystem.getCloseNoteDetected() && shooter.atPitch(shooter.getAutoAngle(shooterLimelight.y, shooterLimelight.a))){
      shooter.setFeeder(1);
      shooter.setManual(0);
      shotNote = true;
    }
    
    if (!ShooterSubsystem.getNoteDetected() && !shotNote){
      drive.move(VectorR.fromPolar(visionSpeed, -DriveSubsystem.getYawDegrees() - intakeLimelight.x), MathR.limit(LIMELIGHT_TURN_KP * MathR.getDistanceToAngle(0, shooterLimelight.x - 4), -0.07, 0.7) * -1);
    }
    
    
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return shotNote;
  }
}
 