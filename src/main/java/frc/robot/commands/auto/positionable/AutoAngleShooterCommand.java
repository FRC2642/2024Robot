// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.positionable;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.MathR;

public class AutoAngleShooterCommand extends Command {
  /** Creates a new AutoAngleShooterCommand. */
  ShooterSubsystem shooter;
  LimelightSubsystem shooterLimelight;
  public AutoAngleShooterCommand(ShooterSubsystem shooter, LimelightSubsystem shooterLimelight) {
    this.shooter = shooter;
    this.shooterLimelight = shooterLimelight;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("tilting");
    shooter.tiltToAngle(shooter.getAutoAngle(shooterLimelight.y, shooterLimelight.a));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println(shooter.getAutoAngle(shooterLimelight.y, shooterLimelight.a) + " "+shooter.getPitch());
    //System.out.println(shooter.atPitch(shooter.getAutoAngle(shooterLimelight.y, shooterLimelight.a)));
    
    
    return shooter.atPitch(shooter.getAutoAngle(shooterLimelight.y, shooterLimelight.a));
  }
}
