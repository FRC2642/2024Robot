// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.DriveCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.VectorR;

public class JoystickTurnSpeedDriveCommand extends Command {

  static final double MAX_SPEED = 0.25;

  final DriveSubsystem drive;

  // CONTROLLER DATA
  XboxController control;
  VectorR leftJoystick = new VectorR();
  double rightJoystick = 0.0;

  public JoystickTurnSpeedDriveCommand(DriveSubsystem drive, XboxController control) {
    this.drive = drive;
    this.control = control;
    
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    DriveSubsystem.resetGyro(0.0);
    DriveSubsystem.resetDisplacement(VectorR.fromCartesian(0, 0));
  }

  @Override
  public void execute() {

    leftJoystick.setFromCartesian(control.getLeftX(), -control.getLeftY());
    leftJoystick.rotate(Math.toRadians(-90));
    leftJoystick.pow(2);
    rightJoystick = -1 * Math.pow(control.getRightX(), 2) * Math.signum(control.getRightX());

    if (leftJoystick.getMagnitude() < 0.1 && Math.abs(rightJoystick) < 0.1) {
      drive.stop();
      return;
    }

    leftJoystick.mult(MAX_SPEED);
    drive.move(leftJoystick, rightJoystick * MAX_SPEED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}