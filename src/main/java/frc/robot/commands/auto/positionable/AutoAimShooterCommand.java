// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.positionable;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;


public class AutoAimShooterCommand extends Command {

  private final ShooterSubsystem shooter;
  private final DriveSubsystem drive;
  private final Supplier<ShooterSubsystem.ShooterSpeed> speed;
  private final LimelightSubsystem shooterLimelight;
  
  final double LIMELIGHT_TURN_KP = 0.009;
  double turnPower;
  
  public AutoAimShooterCommand(DriveSubsystem drive, ShooterSubsystem shooter, Supplier<ShooterSubsystem.ShooterSpeed> speed, LimelightSubsystem shooterLimelight) {
    this.shooter = shooter;
    this.speed = speed;
    this.shooterLimelight = shooterLimelight;
    this.drive = drive;
    addRequirements(shooter, shooterLimelight);
  }
  @Override
  public void initialize() {}
  
  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public void execute() {
    if (shooterLimelight.isDetection){
      shooter.tiltToAngle(shooter.getAutoAngle(shooterLimelight.y));
    }
    else{
      shooter.setManual(0);
    }
    
    shooter.set(speed.get());
    turnPower = MathR.limit(LIMELIGHT_TURN_KP * MathR.getDistanceToAngle(0, shooterLimelight.x - 4), -0.19, 0.19) * -1;
    drive.move(new VectorR(), turnPower);
  }

  @Override
  public boolean isFinished() {
    //System.out.println("angle: "+angle.get().angle + " pitch: "+shooter.getPitch());
  
    boolean robotReady = Math.abs(turnPower) <= 0.05;
    //boolean shooterRevReady = (ShooterSubsystem.getMotorVelocity() <= -70) && (ShooterSubsystem.getMotorVelocity() >= -94);
    boolean shooterAngleReady = shooter.atPitch(shooter.getAutoAngle(shooterLimelight.y));
    System.out.println("robot: " + robotReady);
    //System.out.println("shooter: "+shooterAngleReady);
    //System.out.println("shooter rev: "+shooterRevReady);
    
    System.out.println(shooter.getPitch() + " "+shooter.getAutoAngle(shooterLimelight.y));
    
    
    
    return shooterAngleReady && robotReady;
  }
}
