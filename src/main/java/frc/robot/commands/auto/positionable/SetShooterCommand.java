// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.positionable;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


public class SetShooterCommand extends Command {

  private final ShooterSubsystem shooter;
  private final Supplier<ShooterSubsystem.ShooterPosition> position;
  
  public SetShooterCommand(ShooterSubsystem shooter, Supplier<ShooterSubsystem.ShooterPosition> position) {
    this.shooter = shooter;
    this.position = position;
    addRequirements(shooter);
  }
  @Override
  public void initialize() {
    shooter.setSpeedLimit(0.3);
    shooter.setRampRate(0);
  }
  
  @Override
  public void end(boolean interrupted) {
    shooter.set(0.0);
  }
  
  @Override
  public void execute() {
    shooter.set(position.get());
  }

  @Override
  public boolean isFinished() {
    return shooter.atSetPosition();  
  }
}
