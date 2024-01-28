// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.DriveCommands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevatorCommand extends Command {
  /** Creates a new ManualElevatorCommand. */

  private final ElevatorSubsystem elevator;
  private final Joystick auxButtonBoard;
  double speed;

  public ManualElevatorCommand(ElevatorSubsystem elevator, Joystick auxButtonBoard){
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.auxButtonBoard = auxButtonBoard;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(auxButtonBoard.getRawButton(4)){
      speed = 1;
    }
    else if(auxButtonBoard.getRawButton(5)){
      speed = -1;
    }
    else{
      speed = 0;
    }

    elevator.set(speed);
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
