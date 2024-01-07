// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class DriveTowardsGamePieceCommand extends Command {

  DriveSubsystem drive;
  LimelightSubsystem limelight;
  LimelightSubsystem.DetectionType type;
  double speed;

  final VectorR velocity = new VectorR();

  public DriveTowardsGamePieceCommand(DriveSubsystem drive, LimelightSubsystem limelight, LimelightSubsystem.DetectionType type, double speed) {
    this.drive = drive;
    this.limelight = limelight;
    this.type = type;
    this.speed = speed;
    addRequirements(drive, limelight);
  }

  @Override
  public void initialize() {
    limelight.setDetectionType(type);
  }

  @Override
  public void execute() {
    velocity.setFromPolar(speed, DriveSubsystem.getYawDegrees() + limelight.x + 180);

    
    limelight.setDetectionType(type);


    if (limelight.isDetection && limelight.confidence() > 0.25) drive.move(velocity, MathR.limit(limelight.x * -1 * (1d/70d), -0.25, 0.25) );
    
    else{
      drive.move(VectorR.fromPolar(0.1, 0),  MathR.limit(limelight.x * -1 * (1d/45d), -0.25, 0.25));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
