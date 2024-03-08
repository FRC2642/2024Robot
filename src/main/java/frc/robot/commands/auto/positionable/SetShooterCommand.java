// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.positionable;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;


public class SetShooterCommand extends Command {

  private final ShooterSubsystem shooter;
  private final Supplier<ShooterSubsystem.ShooterAngle> angle;
  private final Supplier<ShooterSubsystem.ShooterSpeed> speed;
  
  public SetShooterCommand(ShooterSubsystem shooter, Supplier<ShooterSubsystem.ShooterAngle> angle, Supplier<ShooterSubsystem.ShooterSpeed> speed) {
    this.shooter = shooter;
    this.angle = angle;
    this.speed = speed;
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
    shooter.tiltToAngle(angle.get().angle);
    shooter.set(speed.get());
  }

  @Override
  public boolean isFinished() {
    //System.out.println("angle: "+angle.get().angle + " pitch: "+shooter.getPitch());
    
    
    return shooter.getPitch() >= angle.get().angle - 3 && shooter.getPitch() <= angle.get().angle + 3;
  }
}
