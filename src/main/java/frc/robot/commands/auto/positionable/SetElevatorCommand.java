// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.positionable;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


public class SetElevatorCommand extends Command {

  private final ElevatorSubsystem elevator;
  private final Supplier<ElevatorSubsystem.ElevatorPosition> position;
  
  public SetElevatorCommand(ElevatorSubsystem elevator, Supplier<ElevatorSubsystem.ElevatorPosition> position) {
    this.elevator = elevator;
    this.position = position;
    addRequirements(elevator);
  }
  @Override
  public void initialize() {
    elevator.setSpeedLimit(0.5);
    elevator.setRampRate(0.5);
  }
  
  @Override
  public void end(boolean interrupted) {
    elevator.set(0.0);
  }
  
  @Override
  public void execute() {
    elevator.set(position.get());
  }

  @Override
  public boolean isFinished() {
    return elevator.atSetPosition();  
  }
}
