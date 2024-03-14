// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.positionable;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class AutoAimShooterCommand extends Command {

  private final ShooterSubsystem shooter;
  private final Supplier<ShooterSubsystem.ShooterSpeed> speed;
  private final LimelightSubsystem shooterLimelight;
  
  public AutoAimShooterCommand(ShooterSubsystem shooter, Supplier<ShooterSubsystem.ShooterSpeed> speed, LimelightSubsystem shooterLimelight) {
    this.shooter = shooter;
    this.speed = speed;
    this.shooterLimelight = shooterLimelight;
    addRequirements(shooter);
  }
  @Override
  public void initialize() {
    shooter.setSpeedLimit(0.3);
  }
  
  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public void execute() {
    shooter.tiltToAngle(shooter.getAutoAngle(shooterLimelight.y));
    shooter.set(speed.get());
  }

  @Override
  public boolean isFinished() {
    //System.out.println("angle: "+angle.get().angle + " pitch: "+shooter.getPitch());
    
    
    return shooter.getPitch() >= shooter.getAutoAngle(shooterLimelight.y) - 3 && shooter.getPitch() <= shooter.getAutoAngle(shooterLimelight.y) + 3;
  }
}
