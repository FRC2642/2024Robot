// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotState;

public class PresetSelectorCommand extends Command {
  private final Joystick auxButtonBoard;
  private final XboxController mainControl;


  public PresetSelectorCommand(XboxController mainControl, Joystick auxButtonBoard){
    this.auxButtonBoard = auxButtonBoard;
    this.mainControl = mainControl;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (auxButtonBoard.getRawButtonPressed(0)){
      RobotState.setChosenConfiguration(RobotState.RobotConfiguration.SHOOT_SPEAKER);
    }
    else if (auxButtonBoard.getRawButtonPressed(1)){
      RobotState.setChosenConfiguration(RobotState.RobotConfiguration.SHOOT_AMP);
    }
    else if (auxButtonBoard.getRawButtonPressed(2)){
      RobotState.setChosenConfiguration(RobotState.RobotConfiguration.SHOOT_TRAP);
    }
    else if (mainControl.getLeftBumper()){
      RobotState.setChosenConfiguration(RobotState.RobotConfiguration.INTAKE);
    }
    else{
      RobotState.setRobotState(RobotState.RobotConfiguration.TRAVEL);
    }

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}