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
  
  public SetShooterCommand(ShooterSubsystem shooter, Supplier<ShooterSubsystem.ShooterAngle> angle) {
    this.shooter = shooter;
    this.angle = angle;
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
    shooter.tiltToAngle(angle.get().angle);
    
  }

  @Override
  public boolean isFinished() {
    //System.out.println("angle: "+angle.get().angle + " pitch: "+shooter.getPitch());
    
    if (shooter.getPitch() >= angle.get().angle - 3 && shooter.getPitch() <= angle.get().angle + 3){
      shooter.setManual(0);
    }
    
    return shooter.getPitch() >= angle.get().angle - 3 && shooter.getPitch() <= angle.get().angle + 3;
  }
}
