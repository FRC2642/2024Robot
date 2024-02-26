// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.DriveCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class JoystickOrientedDriveCommand extends CommandBase {

  private double maxSpeed = 0.25;

  private final DriveSubsystem drive;
  private final XboxController control;
  private final VectorR leftJoystick = new VectorR();
  private final VectorR rightJoystick = new VectorR();

  

  public JoystickOrientedDriveCommand(DriveSubsystem drive, XboxController control) {
    this.drive = drive;
    this.control = control;
    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  final double TURN_KP = 0.01;//0.017;
  private boolean isLocked = false;
  private double lockedHeading = 0;

  @Override
  public void execute() {
      maxSpeed = MathR.lerp(0.25, 1.0, 0.0, 1.0, control.getLeftTriggerAxis());

      leftJoystick.setFromCartesian(control.getLeftX(), -control.getLeftY());
      leftJoystick.rotate(-90);
      rightJoystick.setFromCartesian(-control.getRightX(), -control.getRightY());
      rightJoystick.rotate(90);

      double yaw = DriveSubsystem.getYawDegrees();

      if (leftJoystick.getMagnitude() < 0.1 && rightJoystick.getMagnitude() < 0.2) {
        drive.stop();
        isLocked = false;
        return;
      }

      if (leftJoystick.getMagnitude() > 0.1 && rightJoystick.getMagnitude() < 0.2) {
        if (!isLocked) {
          lockedHeading = yaw;
          isLocked = true;
        }
      } 
      else if (leftJoystick.getMagnitude() < 0.1 && rightJoystick.getMagnitude() > 0.2) {
        leftJoystick.setFromCartesian(0.0, 0.0);
      }
      else isLocked = false;

      double angleToFace = isLocked ? lockedHeading : rightJoystick.getAngle();

    
      
      double turnPower = MathR.lerp(0.35, 1, 0.2, 1.0, rightJoystick.getMagnitude())  * MathR
          .limit(TURN_KP * MathR.getDistanceToAngle(yaw, angleToFace), -1, 1);

      System.out.println(yaw + " " + angleToFace);
      
      
      leftJoystick.mult(maxSpeed);
      drive.move(leftJoystick, turnPower * maxSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}