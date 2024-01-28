// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeCommand extends Command {
  IntakeSubsystem intake;
  double speed;
  XboxController mainControl;
  public RunIntakeCommand(IntakeSubsystem intake, double speed, XboxController mainControl) {
    this.intake = intake;
    this.speed = speed;
    this.mainControl = mainControl;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setIntake(speed);
  }

  @Override
  public boolean isFinished() {
    return !mainControl.getLeftBumper();
  }
}
