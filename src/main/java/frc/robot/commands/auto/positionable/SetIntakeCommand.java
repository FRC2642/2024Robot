// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.positionable;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class SetIntakeCommand extends Command {

  private final IntakeSubsystem intake;
  private final Supplier<IntakeSubsystem.IntakePosition> position;
  
  public SetIntakeCommand(IntakeSubsystem intake, Supplier<IntakeSubsystem.IntakePosition> position) {
    this.intake = intake;
    this.position = position;
    addRequirements(intake);
  }
  @Override
  public void initialize() {
    intake.setSpeedLimit(0.9);
    intake.setRampRate(0);
  }
  
  @Override
  public void end(boolean interrupted) {
    intake.set(0.0);
  }
  
  @Override
  public void execute() {
    intake.set(position.get());
  }

  @Override
  public boolean isFinished() {
    return intake.atSetPosition();  
  }
}
