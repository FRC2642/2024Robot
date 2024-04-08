// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.MathR;

public class LimelightPathCommand extends FollowPathCommand {
  /** Creates a new LimelightPathCommand. */
  LimelightSubsystem shooterLimelight;
  final double LIMELIGHT_TURN_KP = 0.009;
  public LimelightPathCommand(DriveSubsystem drive, PiratePath path, boolean recenterDisplacementToFirstPoint, double additionalLookaheadTime, LimelightSubsystem shooterLimelight) {
    super(drive, path, recenterDisplacementToFirstPoint, additionalLookaheadTime);
    this.shooterLimelight = shooterLimelight;
    addRequirements(shooterLimelight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!isStarted()) {
      drive.stop();
      return;
    }
  
    

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
    

    double turn = MathR.limit(LIMELIGHT_TURN_KP * MathR.getDistanceToAngle(0, shooterLimelight.x), -0.2, 0.2) * -1;

    drive.move(velocity, turn * HEADING_KP);
  }

}
