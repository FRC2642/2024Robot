// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.RunningAverage;
import frc.robot.utils.VectorR;

public class TurnTowardsVisionCommand extends Command {
  DriveSubsystem drive;
  LimelightSubsystem limelight;
  XboxController xbox;
  public static final double CONFIDENCE = 0.8;
  PIDController lineUpPID = new PIDController(0.2, 0, 0);

  VectorR leftJoystick = new VectorR();
 // Iterator<DetectionType> detectionIterator =  Arrays.stream(DetectionType.values()).iterator();
  Timer timer = new Timer();
  LimelightSubsystem.DetectionType pipeline;
  private final RunningAverage leftAvg = new RunningAverage(20, 1.2);
  private final RunningAverage rightAvg = new RunningAverage(20, -1.2);
  /** Creates a new TurnTowardsVisionCommand. */
  public TurnTowardsVisionCommand(DriveSubsystem drive, LimelightSubsystem limelight, XboxController xbox, LimelightSubsystem.DetectionType pipeline) {
    this.drive = drive;
    this.limelight = limelight;
    this.xbox = xbox;
    this.pipeline = pipeline;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelight.setDetectionType(pipeline);
    
    if (limelight.isDetection == true && limelight.botposeZRot > 0){
      leftAvg.add(limelight.botposeZRot);
    }
    else if (limelight.isDetection == true && limelight.botposeZRot < 0){
      rightAvg.add(limelight.botposeZRot);
    }
    
    leftJoystick.setFromCartesian(xbox.getLeftX(),- xbox.getLeftY());
    leftJoystick.rotate(Math.toRadians(90));
    double maxSpeed = MathR.lerp(0.25, 1.0, 0.0, 1.0, xbox.getLeftTriggerAxis());
    leftJoystick.mult(maxSpeed);

    if (limelight.getDetectionType() != LimelightSubsystem.DetectionType.FIDUCIAL){
      if (limelight.confidence() > CONFIDENCE) drive.move(leftJoystick, MathR.limit(-limelight.x * (1d/27d), -1, 1) * 0.4  );
      else drive.move(leftJoystick, xbox.getRightX());
    }
    else{
      if (limelight.botposeZRot < -6){
        drive.move(VectorR.fromPolar(MathR.limit(lineUpPID.calculate(rightAvg.get(), 0), -0.4, 0.4) * 0.15, 3*Math.PI/2), MathR.limit(-limelight.x * (1d/27d), -1, 1) * 0.3);
      }
      else if (limelight.botposeZRot > 6){
        drive.move(VectorR.fromPolar(-MathR.limit(lineUpPID.calculate(leftAvg.get(), 0), -0.4, 0.4) * 0.15, Math.PI/2), MathR.limit(-limelight.x * (1d/27d), -1, 1) * 0.3);
      }
      else{
        if (limelight.a < 1.4){
          drive.move(VectorR.fromPolar(0.1, 0), MathR.limit(-limelight.x * (1d/27d), -1, 1) * 0.3);
        }
        else{
          drive.stop();
        }
      }
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
