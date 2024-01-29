// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.positionable;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;
import frc.robot.subsystems.RobotState.RobotConfiguration;

public class RobotPresetCommand extends Command {
  /** Creates a new RobotPresetCommand. */
  DriveSubsystem drive;
  ShooterSubsystem shooter;
  ElevatorSubsystem elevator;
  IntakeSubsystem intake;
  LimelightSubsystem intakeLimelight;
  LimelightSubsystem shooterLimelight;
  XboxController control;
  Joystick auxButtonBoard;

  final VectorR leftJoystick = new VectorR();
  private final VectorR rightJoystick = new VectorR();
  final double TURN_KP = 0.017;
  private double maxSpeed = 0.25;
  private boolean isLocked = false;
  private double lockedHeading = 0;

  public RobotPresetCommand(DriveSubsystem drive, ShooterSubsystem shooter, ElevatorSubsystem elevator, IntakeSubsystem intake, LimelightSubsystem intakeLimelight, LimelightSubsystem shooterLimelight, XboxController control, Joystick auxButtonBoard) {
    this.drive = drive;
    this.shooter = shooter;
    this.elevator = elevator;
    this.intake = intake;
    this.intakeLimelight = intakeLimelight;
    this.shooterLimelight = shooterLimelight;
    this.control = control;
    this.auxButtonBoard = auxButtonBoard;
    addRequirements(drive, shooter, elevator, intake, intakeLimelight, shooterLimelight);
  }

  @Override
  public void initialize() {
    shooter.setSpeedLimit(0.5);
    shooter.setRampRate(0.2);
    elevator.setSpeedLimit(0.5);
    elevator.setRampRate(0.5);
    intake.setSpeedLimit(0.5);
    intake.setRampRate(0.2);
  }

  @Override
  public void execute() {
    shooter.setSpeedLimit(0.9);
    shooter.setRampRate(0);
    intake.setSpeedLimit(0.9);
    intake.setRampRate(0);
    elevator.setSpeedLimit(0.5);
    elevator.setRampRate(0.5);

    //Set subsystem presets
    if (control.getLeftBumper() || control.getRightBumper()){
      intake.set(RobotState.getChosenRobotConfiguration().intakePos);
      elevator.set(RobotState.getChosenRobotConfiguration().elevatorPos);
      
      if (!RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_SPEAKER)){
        shooter.set(RobotState.getChosenRobotConfiguration().shooterPos);
      }
    }
    else{
      RobotState.setRobotState(RobotConfiguration.TRAVEL);
      intake.set(RobotState.getRobotConfiguration().intakePos);
      elevator.set(RobotState.getRobotConfiguration().elevatorPos);
      shooter.set(RobotState.getRobotConfiguration().shooterPos);
    }

    

    

    //SET SHOOTER FOR REQUIRED PRESETS
    if (!(RobotState.getRobotConfiguration().equals(RobotConfiguration.INTAKE) || RobotState.getRobotConfiguration().equals(RobotConfiguration.TRAVEL))){
      shooter.setShooter(0.7);
      if (control.getRightTriggerAxis() >= 0.1){
        shooter.setFeeder(1);
      } 
      else{
        shooter.setFeeder(0);
      }
    }
    else{
      shooter.setShooter(0);
    }

    //INTAKE PRESET
    if (RobotState.getRobotConfiguration().equals(RobotConfiguration.INTAKE)){
      intake.setIntake(0.7);
      intakeLimelight.setDetectionType(DetectionType.NOTE);

      leftJoystick.setFromCartesian(control.getLeftX(), -control.getLeftY());
      leftJoystick.rotate(-90);
      rightJoystick.setFromCartesian(control.getRightX(), -control.getRightY());
      rightJoystick.rotate(90);

      leftJoystick.mult(MathR.lerp(0.25, 1.0, 0.0, 1.0, control.getLeftTriggerAxis()));

      double angleToFace = isLocked ? lockedHeading : rightJoystick.getAngle();

      double turnPower = MathR.lerp(0.35, 1, 0.2, 1.0, rightJoystick.getMagnitude())  * MathR
          .limit(TURN_KP * MathR.getDistanceToAngle(DriveSubsystem.getYawDegrees(), angleToFace), -1, 1);

      if (intakeLimelight.isDetection && intakeLimelight.confidence() > 0.25) drive.move(leftJoystick, MathR.limit(intakeLimelight.x * -1 * (1d/45d), -0.25, 0.25) );
      else if (leftJoystick.getMagnitude() > 0.1) drive.move(leftJoystick, turnPower * maxSpeed);
    }
    else{
      intake.setIntake(0);
    }

    //SPEAKER PRESET
    if (RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_SPEAKER)){ 
      shooter.setShooter(0.7);
      if (control.getRightTriggerAxis() >= 0.1){
        shooter.setFeeder(1);
      } else{shooter.set(0);}

      leftJoystick.setFromCartesian(control.getLeftX(), -control.getLeftY());
      leftJoystick.rotate(-90);
    
      shooterLimelight.setDetectionType(DetectionType.FIDUCIAL);

      leftJoystick.mult(MathR.lerp(0.25, 1.0, 0.0, 1.0, control.getLeftTriggerAxis()));

      double distanceToSpeaker = Math.sqrt(Math.pow(4.5416 - shooterLimelight.botposeX, 2) + Math.pow(shooterLimelight.botposeY, 2));
      double angleToSpeaker = Math.toDegrees(Math.atan2(Constants.SPEAKER_TARGET_HEIGHT - elevator.getHeight(), distanceToSpeaker));

      if (shooterLimelight.isDetection && shooterLimelight.confidence() > 0.1){
        shooter.tiltToAngle(angleToSpeaker);
      }
      else{
        shooter.tiltToAngle(60);
      }
      

      VectorR direction = DriveSubsystem.getRelativeVelocity();
      direction.div(DriveSubsystem.getRelativeVelocity().getMagnitude());
      
      double angleToFace = shooterLimelight.x + 90 - Math.toDegrees(Math.atan2(Constants.SHOOTER_VELOCITY, DriveSubsystem.getRelativeVelocity().getX() + 0.0001));

      double turnPower = MathR.limit(TURN_KP * MathR.getDistanceToAngle(0, angleToFace), -0.25, 0.25) * -1;
      
      if (shooterLimelight.isDetection && shooterLimelight.confidence() > 0.2) drive.move(leftJoystick, turnPower);
      else if (leftJoystick.getMagnitude() > 0.1) drive.move(leftJoystick, 0.0);
    }
    
    //TRAVEL MODE AND DRIVE FOR TRAP AND AMP PRESETS
    if (RobotState.getRobotConfiguration().equals(RobotConfiguration.TRAVEL) || RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_AMP) || RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_TRAP)){ 
      maxSpeed = MathR.lerp(0.25, 1.0, 0.0, 1.0, control.getLeftTriggerAxis());

      leftJoystick.setFromCartesian(control.getLeftX(), -control.getLeftY());
      leftJoystick.rotate(-90);
      rightJoystick.setFromCartesian(control.getRightX(), -control.getRightY());
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

      leftJoystick.mult(maxSpeed);
      drive.move(leftJoystick, turnPower * maxSpeed);
    }

    //CLIMB PRESET
    if (auxButtonBoard.getRawButton(3)){
      elevator.set(ElevatorPosition.CLIMB);
    }
    else{
      elevator.set(RobotState.getRobotConfiguration().elevatorPos);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
