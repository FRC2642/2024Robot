// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.RobotState.RobotConfiguration;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;

public class IntakeUntilFound extends Command {
  /** Creates a new IntakeUntilFound. */
  IntakeSubsystem intake;
  ShooterSubsystem shooter;
  RobotConfiguration configuration;
  Supplier<IntakeSubsystem.IntakePosition> position;
  Supplier<ShooterSubsystem.ShooterAngle> angle;
  boolean revShooter;
  LimelightSubsystem shooterLimelight;
  boolean runShooter;
  public IntakeUntilFound(Supplier<IntakeSubsystem.IntakePosition> position, IntakeSubsystem intake, ShooterSubsystem shooter, boolean runShooter) {
    this.intake = intake;
    this.shooter = shooter;
    this.position = position;       
    this.runShooter = runShooter;                                                                                                                                      
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.set(position.get());
    shooter.tiltToAngle(ShooterAngle.TRAVEL.angle);
    
    intake.setIntake(1);
    if (!runShooter){
      shooter.stopShooter();
    }
  
    if (!(ShooterSubsystem.getNoteDetected() || ShooterSubsystem.getCloseNoteDetected())){
      shooter.setFeeder(0.9);
    } 
    else if (ShooterSubsystem.getCloseNoteDetected() && !ShooterSubsystem.getNoteDetected()){
      shooter.setFeeder(0.2);
    }
        
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (ShooterSubsystem.getNoteDetected()){
      shooter.setFeeder(0);
      intake.setIntake(0);
    }
    return ShooterSubsystem.getNoteDetected();
  }
}
