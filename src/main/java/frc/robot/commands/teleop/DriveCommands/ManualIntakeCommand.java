// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.DriveCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualIntakeCommand extends Command {
  /** Creates a new ManualIntakeCommand. */

  private final IntakeSubsystem intake;
  private final XboxController mainControl;
  double speed;


  public ManualIntakeCommand(IntakeSubsystem intake, XboxController mainControl) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.intake = intake;
    this.mainControl = mainControl;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mainControl.getLeftBumper()){
      speed = 1;
    }
    else{
      speed = 0;
    }

    intake.set(speed);
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
