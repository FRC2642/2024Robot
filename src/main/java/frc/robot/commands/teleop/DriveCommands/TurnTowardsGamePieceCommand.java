// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.DriveCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class TurnTowardsGamePieceCommand extends Command {

  DriveSubsystem drive;
  XboxController control;
  LimelightSubsystem limelight;
  LimelightSubsystem.DetectionType type;

  final VectorR leftJoystick = new VectorR();

  public TurnTowardsGamePieceCommand(DriveSubsystem drive, LimelightSubsystem limelight, LimelightSubsystem.DetectionType type, XboxController control) {
    this.drive = drive;
    this.limelight = limelight;
    this.type = type;
    this.control = control;
    addRequirements(drive, limelight);
  }

  @Override
  public void initialize() {
    limelight.setDetectionType(type);
  }

  @Override
  public void execute() {
    leftJoystick.setFromCartesian(control.getLeftX(), -control.getLeftY());
    leftJoystick.rotate(-90);

    
    limelight.setDetectionType(type);

    leftJoystick.mult(MathR.lerp(0.25, 1.2, 0.0, 1.0, control.getLeftTriggerAxis()));

    if (limelight.isDetection && limelight.confidence() > 0.25) drive.move(leftJoystick, MathR.limit(limelight.x * -1 * (1d/45d), -0.25, 0.25) );
    else if (leftJoystick.getMagnitude() > 0.1) drive.move(leftJoystick, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
