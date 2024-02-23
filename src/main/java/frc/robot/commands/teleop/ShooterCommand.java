// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */

  XboxController control;
  ShooterSubsystem shooter;
  int i;

  public ShooterCommand(XboxController controlIN, ShooterSubsystem shooterIN) {
    this.control = controlIN;
    this.shooter = shooterIN;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterIN);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 10;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (control.getRightTriggerAxis() > 0.02) {
      shooter.runFlyWheels(control.getRightTriggerAxis());
      if (i <= 0) {
        shooter.runBackRoller(control.getRightTriggerAxis());
      } else {
        i -= 1;
      }
    } else {
      i = 10;
      shooter.runFlyWheels(0);
      shooter.runBackRoller(0);
    }
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
