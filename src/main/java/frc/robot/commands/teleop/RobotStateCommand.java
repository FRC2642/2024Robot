// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RobotStateCommand extends ParallelDeadlineGroup {
  /** Creates a new RobotStateCommand. */
  public RobotStateCommand(DriveSubsystem drive, ShooterSubsystem shooter, ElevatorSubsystem elevator, IntakeSubsystem intake, LimelightSubsystem intakeLimelight, LimelightSubsystem shooterLimelight, XboxController control) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    
    super(new PresetSelectorCommand(control));
    if (RobotState.SHOOT){
      addCommands(null);
    }
  }
}
