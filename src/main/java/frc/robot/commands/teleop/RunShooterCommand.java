// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterCommand extends Command {
  /** Creates a new RunShooterCommand. */
  ShooterSubsystem shooter;
  double speed;
  XboxController mainControl;
  public RunShooterCommand(ShooterSubsystem shooter, double speed, XboxController mainControl) {
    this.shooter = shooter;
    this.speed = speed;
    this.mainControl = mainControl;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooter(speed);
    if (mainControl.getRightTriggerAxis() >= 0.1){
      shooter.setFeeder(1);
    }
    else{
      shooter.setFeeder(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !mainControl.getRightBumper();
  }
}
