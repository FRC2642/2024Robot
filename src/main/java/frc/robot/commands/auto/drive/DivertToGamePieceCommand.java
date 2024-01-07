// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class DivertToGamePieceCommand extends FollowPathCommand {
  /** Creates a new DivertToGamePieceCommand. */
  DriveSubsystem drive;
  LimelightSubsystem limelight;
  LimelightSubsystem.DetectionType object;
  PiratePath path;
  double visionSpeed;
  double timeAfterStartToDivert;
  Timer visionTimer = new Timer();
  Timer intakeTimer = new Timer();
  boolean endWhenIntaken;
  

  public DivertToGamePieceCommand(DriveSubsystem drive, LimelightSubsystem limelight, LimelightSubsystem.DetectionType object, PiratePath path, boolean recenterDisplacementToFirstPoint, double additionalLookaheadTime, double visionSpeed, double timeAfterStartToDivert,  boolean endWhenIntaken) {
    super(drive, path, recenterDisplacementToFirstPoint, additionalLookaheadTime);
    this.drive = drive;
    this.endWhenIntaken = endWhenIntaken;
    this.limelight = limelight;
    this.object = object;
    this.path = path;
    this.visionSpeed = visionSpeed;
    this.timeAfterStartToDivert = timeAfterStartToDivert;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    limelight.setDetectionType(object);
    visionTimer.reset();
    visionTimer.start();
    intakeTimer.reset();
    intakeTimer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double addedAngle = 0;
    if (endWhenIntaken){
      addedAngle = 180;
    }
    
    if (visionTimer.get() > timeAfterStartToDivert && limelight.isDetection && limelight.confidence() > 0.25){
      
      drive.move(VectorR.fromPolar(visionSpeed, DriveSubsystem.getYawDegrees() + limelight.x + addedAngle), MathR.limit(limelight.x * -1 * (1d/90d), -0.20, 0.20));
    }
    else if (super.isFinished()){
      drive.move(VectorR.fromPolar(0.20, DriveSubsystem.getYawDegrees() + limelight.x + addedAngle), MathR.limit(limelight.x * -1 * (1d/90d), -0.20, 0.20));
    }

    else{
      super.execute();
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endWhenIntaken){
      return intakeTimer.get() > 0.15 /*&& ClawIntakeSubsystem.isObjectInClaw()*/;
    }
    else return super.isFinished();
  }
}
 