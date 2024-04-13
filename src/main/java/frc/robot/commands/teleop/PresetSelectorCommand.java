// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.RobotState.RobotConfiguration;

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
    if (auxButtonBoard.getRawButtonPressed(11)){
      RobotState.setChosenConfiguration(RobotState.RobotConfiguration.SHOOT_SPEAKER);
    }
    else if (auxButtonBoard.getRawButtonPressed(12)){
      RobotState.setChosenConfiguration(RobotState.RobotConfiguration.SHOOT_AMP);
    }
    else if (auxButtonBoard.getRawButtonPressed(8)){
      RobotState.setChosenConfiguration(RobotState.RobotConfiguration.SHOOT_OVER);
    }
    else if (auxButtonBoard.getRawButtonPressed(10)){
      RobotState.setChosenConfiguration(RobotState.RobotConfiguration.PASS);
    }
    else if (auxButtonBoard.getRawButtonPressed(7)){
      RobotState.setChosenConfiguration(RobotState.RobotConfiguration.TRAP);
    }
    else if (mainControl.getLeftBumper()){
      RobotState.setRobotState(RobotState.RobotConfiguration.INTAKE);
    }
    else if (mainControl.getRightBumper()){
      RobotState.setRobotState(RobotState.getChosenRobotConfiguration());
    }
    else if (mainControl.getRawButton(8)){
      RobotState.setRobotState(RobotConfiguration.CLIMB_UP);
    }
    else if (mainControl.getRawButton(7)){
      RobotState.setRobotState(RobotConfiguration.CLIMB_DOWN);
    }
    else if (!(RobotState.getRobotConfiguration().equals(RobotConfiguration.CLIMB_UP) || RobotState.getRobotConfiguration().equals(RobotConfiguration.CLIMB_DOWN))){
      RobotState.setRobotState(RobotState.RobotConfiguration.TRAVEL);
    }

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
