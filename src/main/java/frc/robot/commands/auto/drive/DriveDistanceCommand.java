// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.VectorR;

public class DriveDistanceCommand extends DriveDirectionCommand {
  
  private final double distance;

  public DriveDistanceCommand(DriveSubsystem drive, VectorR velocity, double faceDegree, double distance) {
    super(drive, velocity, faceDegree);
    this.distance = distance;
  }
  
  public DriveDistanceCommand(DriveSubsystem drive, VectorR velocity, double distance) {
    super(drive, velocity);
    this.distance = distance;
  }
  
  @Override
  public boolean isFinished() {
    return localDisplacement.getMagnitude() > distance;
  }
}
