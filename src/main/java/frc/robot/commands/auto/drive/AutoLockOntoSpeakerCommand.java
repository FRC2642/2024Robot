// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class AutoLockOntoSpeakerCommand extends FollowPathCommand {
  final double TURN_KP = 0.017;

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

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      limelight.setDetectionType(object);

      double distanceToSpeaker = Math.sqrt(Math.pow(4.5416 - limelight.botposeX, 2) + Math.pow(limelight.botposeY, 2));
      double angleToSpeaker = Math.toDegrees(Math.atan2(Constants.SPEAKER_TARGET_HEIGHT - elevator.getHeight(), distanceToSpeaker));

      shooter.tiltToAngle(angleToSpeaker);

      VectorR direction = DriveSubsystem.getRelativeVelocity();
      direction.div(DriveSubsystem.getRelativeVelocity().getMagnitude());
      
      double angleToFace = limelight.x + 90 - Math.toDegrees(Math.atan2(Constants.SHOOTER_VELOCITY - DriveSubsystem.getRelativeVelocity().getY(), DriveSubsystem.getRelativeVelocity().getX() + 0.0001));

      double turnPower = MathR.limit(TURN_KP * MathR.getDistanceToAngle(0, angleToFace), -0.25, 0.25) * -1;
      
      //if (limelight.isDetection && limelight.confidence() > 0.2) drive.move(leftJoystick, turnPower);
      //else if (leftJoystick.getMagnitude() > 0.1) drive.move(leftJoystick, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
