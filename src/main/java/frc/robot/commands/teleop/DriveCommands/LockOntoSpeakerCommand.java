// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.DriveCommands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class LockOntoSpeakerCommand extends TurnTowardsGamePieceCommand {
  final double TURN_KP = 0.017;

  private ShooterSubsystem shooter;
  public LockOntoSpeakerCommand(DriveSubsystem drive, ShooterSubsystem shooter, LimelightSubsystem limelight, DetectionType type, XboxController control) {
    super(drive, limelight, type, control);
    this.shooter = shooter;
    addRequirements(shooter, limelight);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    leftJoystick.setFromCartesian(control.getLeftX(), -control.getLeftY());
      leftJoystick.rotate(-90);
    
      limelight.setDetectionType(type);

      leftJoystick.mult(MathR.lerp(0.25, 1.2, 0.0, 1.0, control.getLeftTriggerAxis()));

      double distanceToSpeaker = Math.sqrt(Math.pow(4.5416 - limelight.botposeX, 2) + Math.pow(limelight.botposeY, 2));
      double angleToSpeaker = Math.toDegrees(Math.atan2(Constants.SPEAKER_TARGET_HEIGHT, distanceToSpeaker));

      shooter.tiltToAngle(angleToSpeaker);

      VectorR direction = DriveSubsystem.getRelativeVelocity();
      direction.div(DriveSubsystem.getRelativeVelocity().getMagnitude());
      
      double angleToFace = limelight.x + 90 - Math.toDegrees(Math.atan2(Constants.SHOOTER_VELOCITY - DriveSubsystem.getRelativeVelocity().getY(), DriveSubsystem.getRelativeVelocity().getX() + 0.0001));

      double turnPower = MathR.limit(TURN_KP * MathR.getDistanceToAngle(0, angleToFace), -0.25, 0.25) * -1;
      
      if (limelight.isDetection && limelight.confidence() > 0.2) drive.move(leftJoystick, turnPower);
      else if (leftJoystick.getMagnitude() > 0.1) drive.move(leftJoystick, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
