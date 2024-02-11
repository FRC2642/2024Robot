// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;


public class AutoLockOntoSpeakerCommand extends FollowPathCommand {
  private final double TURN_KP = 0.017;
  private final Timer timer = new Timer();
  public final Double startingLookAheadTime;

  DriveSubsystem drive;
  LimelightSubsystem limelight;
  LimelightSubsystem.DetectionType object;
  PiratePath path;
  ElevatorSubsystem elevator;
  ShooterSubsystem shooter;

  /** Creates a new AutoLockOntoSpeakerCommand. */
  public AutoLockOntoSpeakerCommand(DriveSubsystem drive, LimelightSubsystem limelight, ElevatorSubsystem elevator, ShooterSubsystem shooter, LimelightSubsystem.DetectionType object, PiratePath path, boolean recenterDisplacementToFirstPoint, double additionalLookaheadTime) {
    super(drive, path, recenterDisplacementToFirstPoint, additionalLookaheadTime);
    this.limelight = limelight;
    this.object = object;
    this.elevator = elevator;
    this.shooter = shooter;

    if (additionalLookaheadTime != 0.0) startingLookAheadTime = BASE_PRECISION + additionalLookaheadTime;
    else startingLookAheadTime = null;

    addRequirements(limelight, elevator, shooter);
  }

  
  @Override
  public void initialize() {
    super.initialize();
  }

  
  @Override
  public void execute() {
      if (startingLookAheadTime == null) lookAheadTime = BASE_PRECISION;
      else {
        lookAheadTime = MathR.lerp(startingLookAheadTime, BASE_PRECISION, 0.0, TIME_TO_CORRECT_FROM_START, timer.get());
        lookAheadTime = MathR.limit(lookAheadTime, BASE_PRECISION, startingLookAheadTime);
      }

      currentTime = timer.get() + path.getFirst().time;

      while ((nextPoint == null || nextPoint.time - currentTime < lookAheadTime) && iterator.hasNext())
        nextPoint = iterator.next();

      var delta_t = nextPoint.time - currentTime;
      if (delta_t < lookAheadTime) delta_t = lookAheadTime;

      var velocity = nextPoint.position.clone();
      velocity.sub(DriveSubsystem.getRelativeFieldPosition());
      velocity.mult(MOVEMENT_KP / delta_t);

      double turn = MathR.getDistanceToAngle(DriveSubsystem.getYawDegrees(), nextPoint.holonomicRotation) / delta_t;
      

      limelight.setDetectionType(object);

      double distanceToSpeaker = Math.sqrt(Math.pow(4.5416 - limelight.botposeX, 2) + Math.pow(limelight.botposeY, 2));
      double angleToSpeaker = Math.toDegrees(Math.atan2(Constants.SPEAKER_TARGET_HEIGHT - elevator.getHeight(), distanceToSpeaker));

      shooter.tiltToAngle(angleToSpeaker);

      VectorR direction = DriveSubsystem.getRelativeVelocity();
      direction.div(DriveSubsystem.getRelativeVelocity().getMagnitude());
      
      double angleToFace = limelight.x + 90 - Math.toDegrees(Math.atan2(Constants.SHOOTER_VELOCITY - DriveSubsystem.getRelativeVelocity().getY(), DriveSubsystem.getRelativeVelocity().getX() + 0.0001));

      double turnPower = MathR.limit(TURN_KP * MathR.getDistanceToAngle(0, angleToFace), -0.25, 0.25) * -1;

      //If april tag is seen, follow path movement and rotate to tag
      if (limelight.isDetection && limelight.confidence() > 0.2) drive.move(velocity, turnPower);
      //If not seen, follow path movement and rotation
      else drive.move(velocity, turn * HEADING_KP);
  }
  
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
