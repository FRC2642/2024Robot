// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.drive.DriveDistanceCommand;
import frc.robot.commands.auto.drive.StopCommand;
import frc.robot.path.PiratePath;
import frc.robot.path.PiratePoint;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Easings.Functions;
import frc.robot.utils.VectorR;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveCommand extends SequentialCommandGroup {
  /** Creates a new MoveCommadn. */
  
  public MoveCommand(DriveSubsystem drive) {
    PiratePath path = new PiratePath(false);
    path.add(new PiratePoint(0, 0, 298.74, 0, false));
    path.add(new PiratePoint(2, 0, 298.74, 4, true));
    path.fillWithSubPointsEasing(0.05, Functions.easeOutExpo);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveDistanceCommand(drive, VectorR.fromPolar(0.2, 0), 5),
      new StopCommand(drive)
    );
  }
}