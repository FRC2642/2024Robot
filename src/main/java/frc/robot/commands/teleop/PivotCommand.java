// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotCommand extends Command {
  /** Creates a new PivotCommand. */

  PivotSubsystem pivot;
  XboxController control;

  public PivotCommand(PivotSubsystem pivotIN, XboxController controlIN) {
    this.pivot = pivotIN;
    this.control = controlIN;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivotIN);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
