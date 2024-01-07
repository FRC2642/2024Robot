// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.resetters;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ResetGyroCommand extends Command {

  private final double heading;

  public ResetGyroCommand(double heading) {
    this.heading = heading;
  }

  @Override
  public void execute() {
    DriveSubsystem.resetGyro(heading);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
