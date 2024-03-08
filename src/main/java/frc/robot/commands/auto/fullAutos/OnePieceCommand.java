// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.positionable.SetShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceCommand extends SequentialCommandGroup {
  /** Creates a new OnePieceCommand. */
  public OnePieceCommand(ShooterSubsystem shooter) {
    
    addCommands(
      new InstantCommand(() -> {
        shooter.setSpeedLimit(0.5);
      }, shooter),

        new SetShooterCommand(shooter, ()->ShooterAngle.SUBWOOFER, ()->ShooterSpeed.SPEAKER),
      
        new WaitCommand(0.5),
        new InstantCommand(()->{
          shooter.setFeeder(1);
        }, shooter),

        new WaitCommand(0.3),

        new InstantCommand(()->{
          shooter.setFeeder(0);
        }, shooter)
    );
  }
}
