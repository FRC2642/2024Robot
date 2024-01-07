// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.resetters;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ToggleStopDefensivelyCommand extends Command {

  private final DriveSubsystem drive;
  
  public ToggleStopDefensivelyCommand(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    drive.setDefensiveMode(!drive.getDefensiveMode());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
