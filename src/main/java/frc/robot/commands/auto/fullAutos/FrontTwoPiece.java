// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.fullAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.IntakeUntilFound;
import frc.robot.commands.auto.drive.FollowPathCommand;
import frc.robot.commands.auto.drive.StopCommand;
import frc.robot.commands.auto.positionable.AutoAimShooterCommand;
import frc.robot.path.PiratePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;
import frc.robot.utils.VectorR;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FrontTwoPiece extends SequentialCommandGroup {
  /** Creates a new FrontTwoPiece. */
  public FrontTwoPiece(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, LimelightSubsystem shooterLimelight) {
    PiratePath path = new PiratePath("TwoPiecePath", false);
    var paths = path.getSubPaths();
    var getNote = paths.get(0);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        drive.setDefensiveMode(true);
        intake.setSpeedLimit(0.65);
        shooter.setSpeedLimit(0.5);
      }, drive, intake, shooter),

      new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),
    
      new WaitCommand(0.7),
      new InstantCommand(()->{
        shooter.setFeeder(1);
      }, shooter),

      new WaitCommand(0.3),

      new InstantCommand(()->{
        shooter.setFeeder(0);
      }, shooter),

      
      new FollowPathCommand(drive, getNote, true, 0.25).alongWith(
        new IntakeUntilFound(()->IntakePosition.EXTENDED, intake, shooter, false)
      ).withTimeout(3),

      


      new InstantCommand(()->{
        intake.setIntake(0);
        shooter.setFeeder(-0.2);
        drive.move(new VectorR(), 0);
      }, intake, shooter, drive),

      new WaitCommand(0.3),
      new InstantCommand(()->{
        shooter.setFeeder(0);
      }, shooter),


      new AutoAimShooterCommand(drive, shooter, ()->ShooterSpeed.SPEAKER, shooterLimelight),
      

      new WaitCommand(1),
      new InstantCommand(()->{
        shooter.setFeeder(1);
      }, shooter),

      new WaitCommand(0.3),

      new InstantCommand(()->{
        shooter.setShooter(0);
        shooter.setFeeder(0);
      }, shooter),
      

      new StopCommand(drive)
    );
  }
}
