// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.positionable;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterAngleCommand extends Command {
  /** Creates a new SetShooterAngleCommand. */
  private final ShooterSubsystem shooter;
  private final Supplier<ShooterSubsystem.ShooterAngle> anglePosition;

  public SetShooterAngleCommand(ShooterSubsystem shooter, Supplier<ShooterSubsystem.ShooterAngle> anglePosition) {
    this.shooter = shooter;
    this.anglePosition = anglePosition;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeedLimit(0.3);
    shooter.setRampRate(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.tiltToAngle(anglePosition.get().angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.atSetPosition();
  }
}