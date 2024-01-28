// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.positionable;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PresetSelectorCommand extends Command {
  /** Creates a new PresetSelectorCommand. */
  private final ShooterSubsystem shooter;
  private final ElevatorSubsystem elevator;
  private final IntakeSubsystem intake;
  private final Joystick auxButtonBoard;


  public PresetSelectorCommand(ShooterSubsystem shooter, ElevatorSubsystem elevator, IntakeSubsystem intake, Joystick auxButtonBoard){
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.elevator = elevator;
    this.intake = intake;
    this.auxButtonBoard = auxButtonBoard;
    addRequirements(shooter, elevator, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public void SpeakerPreset(){}

  public void AmpPreset(){}

  public void TrapPreset(){}

  public void IntakePreset(){}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(auxButtonBoard.getRawButton(0)){
      SpeakerPreset();
    }
    if(auxButtonBoard.getRawButton(1)){
      AmpPreset();
    }
    if(auxButtonBoard.getRawButton(2)){
      TrapPreset();
    }
    if(auxButtonBoard.getRawButton(3)){
      IntakePreset();
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
