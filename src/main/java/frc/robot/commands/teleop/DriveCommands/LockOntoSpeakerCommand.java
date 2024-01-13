// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.DriveCommands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.utils.MathR;

public class LockOntoSpeakerCommand extends TurnTowardsGamePieceCommand {
  /** Creates a new LockOntoSpeakerCommand. */
  final double TURN_KP = 0.017;

  public LockOntoSpeakerCommand(DriveSubsystem drive, LimelightSubsystem limelight, DetectionType type, XboxController control) {
    super(drive, limelight, type, control);


  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double yaw = DriveSubsystem.getYawDegrees();

    leftJoystick.setFromCartesian(control.getLeftX(), -control.getLeftY());
    leftJoystick.rotate(-90);
    
    limelight.setDetectionType(type);

    leftJoystick.mult(MathR.lerp(0.25, 1.2, 0.0, 1.0, control.getRightTriggerAxis()));

    double angleToFace = Math.toDegrees(Math.atan2(limelight.y, limelight.x + 0.0001) + Math.atan2(Constants.SHOOTER_VELOCITY, DriveSubsystem.getRelativeVelocity().getMagnitude() + 0.0001) + 2*Math.PI);

    double turnPower = MathR.limit(TURN_KP * MathR.getDistanceToAngle(yaw, angleToFace), -0.25, 0.25);
    //MathR.limit(limelight.x * -1 * (1d/45d), -0.25, 0.25) 
    if (limelight.isDetection && limelight.confidence() > 0.25) drive.move(leftJoystick, turnPower);
    else if (leftJoystick.getMagnitude() > 0.1) drive.move(leftJoystick, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
