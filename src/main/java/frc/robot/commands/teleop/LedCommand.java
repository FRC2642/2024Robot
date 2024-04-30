// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LedSubsystem.LEDColor;

public class LedCommand extends Command {
  /** Creates a new LedCommand. */

  double matchTime = 0;

  public LedCommand(LedSubsystem leds) {
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // matchTime = Timer.getMatchTime();

    // if(matchTime < 30){
    //   LedSubsystem.setLed(0.61);
    // }
    // else if (matchTime < 10){
    //   LedSubsystem.setLed(-0.11);
    // }

    if (ShooterSubsystem.getNoteDetected()){
      LedSubsystem.setColor(LEDColor.BLUE);
    }
    else{
      LedSubsystem.setColor(LEDColor.GREEN);
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