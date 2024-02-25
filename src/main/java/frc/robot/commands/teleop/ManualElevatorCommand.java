// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevatorCommand extends Command {

  private final ElevatorSubsystem elevator;
  private final XboxController mainControl;
  double speed;

  public 
  ManualElevatorCommand(ElevatorSubsystem elevator, XboxController mainControl){
    this.elevator = elevator;
    this.mainControl = mainControl;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(mainControl.getAButton()){
      speed = 0.5;
    }
    else if(mainControl.getBButton()){
      speed = -0.5;
    }
    else{
      speed = 0;
    }

    elevator.setManual(speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
