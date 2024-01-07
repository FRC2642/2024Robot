// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.VectorR;

public class RecenterDisplacementCommand extends Command {

  public static final double RESET_INTERVAL = 0.2;
  public static final double MIN_CONFIDENCE = 10;

  LimelightSubsystem limelight;

  Timer timer = new Timer();

  public RecenterDisplacementCommand(LimelightSubsystem limelight) {
    this.limelight = limelight;
    addRequirements(limelight);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (limelight.confidence() > MIN_CONFIDENCE && timer.get() > RESET_INTERVAL) {
      timer.reset();
      DriveSubsystem.resetDisplacement(VectorR.fromCartesian(limelight.botposeX, limelight.botposeY));
        
    } 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}