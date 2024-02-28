// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

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
  LimelightSubsystem shooterLimelight;
  XboxController control;
  Joystick auxButtonBoard;

  final VectorR leftJoystick = new VectorR();
  private final VectorR rightJoystick = new VectorR();
  final double TURN_KP = 0.017;
  private double maxSpeed = 0.25;
  private boolean isLocked = false;
  private double lockedHeading = 0;
  

  public RobotPresetCommand(DriveSubsystem drive, ShooterSubsystem shooter, ElevatorSubsystem elevator, IntakeSubsystem intake, LimelightSubsystem shooterLimelight, XboxController control, Joystick auxButtonBoard) {
    this.drive = drive;
    this.shooter = shooter;
    this.elevator = elevator;
    this.intake = intake;
    this.shooterLimelight = shooterLimelight;
    this.control = control;
    this.auxButtonBoard = auxButtonBoard;
    addRequirements(drive, shooter, elevator, intake, shooterLimelight/*, intakeLimelight*/);
  }

  @Override
  public void initialize() {
    shooter.setSpeedLimit(0.5);
    shooter.setRampRate(0);
    elevator.setSpeedLimit(0.6);
    intake.setSpeedLimit(0.7);
    intake.setRampRate(0);
    RobotState.setChosenConfiguration(RobotConfiguration.SHOOT_SPEAKER);
  }

  @Override
  public void execute() {
    ///////////////////////////DRIVE///////////////////////////////
    maxSpeed = MathR.lerp(0.25, 1.0, 0.0, 1.0, control.getLeftTriggerAxis());
    leftJoystick.setFromCartesian(control.getLeftX(), -control.getLeftY());
    leftJoystick.rotate(-90);
    rightJoystick.setFromCartesian(control.getRightX(), -control.getRightY());
    rightJoystick.rotate(-90);

    if (leftJoystick.getMagnitude() < 0.1 && rightJoystick.getMagnitude() < 0.2) {
        drive.stop();
        isLocked = false;
        return;
      }
    else if (leftJoystick.getMagnitude() > 0.1 && rightJoystick.getMagnitude() < 0.2) {
      if (!isLocked) {
        lockedHeading = DriveSubsystem.getYawDegrees();
        isLocked = true;
      }
    } 
    else if (leftJoystick.getMagnitude() < 0.1 && rightJoystick.getMagnitude() > 0.2) {
      isLocked = false;
      leftJoystick.setFromCartesian(0.0, 0.0);
    }
    else isLocked = false;

    double angleToFace = isLocked ? lockedHeading : rightJoystick.getAngle();
      
    double turnPower = MathR.lerp(0.35, 1, 0.2, 1.0, rightJoystick.getMagnitude())  * MathR
        .limit(TURN_KP * MathR.getDistanceToAngle(-DriveSubsystem.getYawDegrees(), angleToFace), -1, 1);
    
    leftJoystick.mult(maxSpeed);
    
    drive.move(leftJoystick, turnPower * maxSpeed);
    ////////////////////////////////////////////////////////


    //Set subsystem presets
    intake.set(RobotState.getRobotConfiguration().intakePos);
    shooter.set(RobotState.getRobotConfiguration().shooterSpeed);
    //elevator.set(RobotState.getRobotConfiguration().elevatorPos);
    if (control.getRightBumper()){
      if (!RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_SPEAKER)){
        shooter.set(RobotState.getChosenRobotConfiguration().shooterPos);
      }
    }
    else{
      shooter.set(RobotState.getRobotConfiguration().shooterPos);
    }

    
    //SET SHOOTER FOR REQUIRED PRESETS
    if (!(RobotState.getRobotConfiguration().equals(RobotConfiguration.INTAKE) || RobotState.getRobotConfiguration().equals(RobotConfiguration.TRAVEL))){
      shooter.setShooterRPM();
      
      if (control.getRightTriggerAxis() >= 0.4){
        shooter.setFeeder(1);
      } 
      else{
        shooter.setFeeder(0);
      }
    }
    else{
      shooter.setShooter(0);
      shooter.setFeeder(0);
    }

    //INTAKE PRESET
    if (RobotState.getRobotConfiguration().equals(RobotConfiguration.INTAKE)){
      //if (!shooter.getNoteDetected()){
        intake.setIntake(0.8);
        shooter.setFeeder(0.6);
      /*} 
      else {
        intake.setIntake(0);
        shooter.setFeeder(0);
        //RobotState.setRobotState(RobotConfiguration.TRAVEL); <-TEST
      }*/
    }
    else{
      intake.setIntake(0);
      shooter.setFeeder(0);
    }

    //SPEAKER PRESET
    if (RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_SPEAKER)){ 
      shooterLimelight.setDetectionType(DetectionType.FIDUCIAL);
      
      //Limelight tracking
      double distanceToSpeaker = Math.sqrt(Math.pow(4.5416 - shooterLimelight.botposeX, 2) + Math.pow(shooterLimelight.botposeY, 2));
      double angleToSpeaker = Math.toDegrees(Math.atan2(Constants.SPEAKER_TARGET_HEIGHT - elevator.getHeight(), distanceToSpeaker)) - Constants.SHOOT_ANGLE_OFFSET;
      
      //System.out.println(angleToSpeaker);
      
      if (shooterLimelight.isDetection && shooterLimelight.confidence() > 0.1){
        shooter.tiltToAngle(angleToSpeaker);
      }
      else{
        shooter.setManual(0);
      }
      

      VectorR direction = DriveSubsystem.getRelativeVelocity();
      direction.div(DriveSubsystem.getRelativeVelocity().getMagnitude());
      
      double adjustedAngle = shooterLimelight.x /*+ 90 - Math.toDegrees(Math.atan2(Constants.SHOOTER_VELOCITY - DriveSubsystem.getRelativeVelocity().getY(), DriveSubsystem.getRelativeVelocity().getX() + 0.000001))*/;

      double limelightTurnPower = MathR.limit(TURN_KP * MathR.getDistanceToAngle(0, adjustedAngle), -0.25, 0.25) * -1;
      
      System.out.println(shooterLimelight.x);

      /*if (shooterLimelight.isDetection && shooterLimelight.confidence() > 0.1) drive.move(leftJoystick, limelightTurnPower);
      else if (leftJoystick.getMagnitude() > 0.1)*/ drive.move(leftJoystick, turnPower);
    }

    //CLIMB PRESET
    if (auxButtonBoard.getRawButton(3)){
      //elevator.set(ElevatorPosition.CLIMB);
    }
    else{
      //elevator.set(RobotState.getRobotConfiguration().elevatorPos);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (control.getYButton()){
      shooter.setShooter(-0.3);
    }
    if (control.getXButton()){
      shooter.setFeeder(-0.4);
    }
    return false;
  }
}
