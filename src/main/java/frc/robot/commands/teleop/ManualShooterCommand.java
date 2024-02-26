// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualShooterCommand extends Command {
  private final ShooterSubsystem shooter;
  private final XboxController mainControl;

  /** Creates a new ManualShooterCommand. */
  public ManualShooterCommand(ShooterSubsystem shooter, XboxController mainControl){
    this.shooter = shooter;
    this.mainControl = mainControl;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (mainControl.getRightTriggerAxis() >= 0.1){
      shooter.setShooter(-mainControl.getRightTriggerAxis());
    }
    else if (mainControl.getYButton()){
      shooter.setShooter(0.3);
    }
    else{
      shooter.setShooter(0);
    }
    if (mainControl.getRightBumper()){
      shooter.setFeeder(0.5);
    }
    else if (mainControl.getXButton()){
      shooter.setFeeder(-0.5);
    }
    else{
      shooter.setFeeder(0);
    }
    
    shooter.setManual(mainControl.getRightY() * 0.5);
    

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
