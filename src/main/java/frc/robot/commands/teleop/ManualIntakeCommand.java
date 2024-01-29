// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualIntakeCommand extends Command {
  /** Creates a new ManualIntakeCommand. */

  private final IntakeSubsystem intake;
  private final XboxController mainControl;
  double speed;


  public ManualIntakeCommand(IntakeSubsystem intake, XboxController mainControl) {

    this.intake = intake;
    this.mainControl = mainControl;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(mainControl.getLeftBumper()){
      intake.setIntake(0.7);
    }
    else{
      intake.setIntake(0);
    }

    intake.setManual(mainControl.getLeftY());

    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
