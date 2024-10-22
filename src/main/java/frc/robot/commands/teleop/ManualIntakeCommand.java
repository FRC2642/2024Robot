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
    intake.setIntake(0);
    intake.setManual(0);
    
    if(mainControl.getLeftBumper()){
      intake.setIntake(0.6);
    }
    if (mainControl.getLeftTriggerAxis() >= 0.1){
      intake.setIntake(-1);
    }
    if (mainControl.getPOV() == 270){
      intake.setManual(-0.6);
    }
    else if (mainControl.getPOV() == 90){
      intake.setManual(0.6);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
