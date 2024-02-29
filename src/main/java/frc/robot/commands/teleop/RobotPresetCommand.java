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
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.ShooterSubsystem.ShooterPosition;

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
  private boolean stopped = false;
  

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
    elevator.setSpeedLimit(0.6);
    intake.setSpeedLimit(0.7);
    RobotState.setChosenConfiguration(RobotConfiguration.SUBWOOFER);
  }

  @Override
  public void execute() {
    
    
    ///////////////////////////DRIVE///////////////////////////////
    maxSpeed = MathR.lerp(0.25, 1.0, 0.0, 1.0, control.getLeftTriggerAxis());
    leftJoystick.setFromCartesian(control.getLeftX(), -control.getLeftY());
    leftJoystick.rotate(-90);
    rightJoystick.setFromCartesian(control.getRightX(), -control.getRightY());
    rightJoystick.rotate(-90);

    
    if (leftJoystick.getMagnitude() < 0.2 && rightJoystick.getMagnitude() < 0.2) {
        stopped = true;
      
        drive.stop();
        isLocked = false;
        leftJoystick.setFromCartesian(0, 0);
        rightJoystick.setFromCartesian(0, 0);
      }
    else if (leftJoystick.getMagnitude() > 0.2 && rightJoystick.getMagnitude() < 0.2) {
      stopped = false;
      if (!isLocked) {
        lockedHeading = DriveSubsystem.getYawDegrees();
        isLocked = true;
        rightJoystick.setFromCartesian(0, 0);
      }
    } 
    else if (leftJoystick.getMagnitude() < 0.2 && rightJoystick.getMagnitude() > 0.2) {
      stopped = false;
      isLocked = false;
      leftJoystick.setFromCartesian(0.0, 0.0);
    }
    else {
      isLocked = false;
      stopped = false;
    }

    double angleToFace = isLocked ? lockedHeading : rightJoystick.getAngle();
      
    double turnPower = MathR.lerp(0.35, 1, 0.2, 1.0, rightJoystick.getMagnitude())  * MathR
        .limit(TURN_KP * MathR.getDistanceToAngle(-DriveSubsystem.getYawDegrees(), angleToFace), -1, 1);
    
    leftJoystick.mult(maxSpeed);
    
    if (!stopped){
      drive.move(leftJoystick, turnPower * maxSpeed);
    }
    
    
    ////////////////////////////////////////////////////////


    
    //Set subsystem presets
    intake.set(RobotState.getRobotConfiguration().intakePos);
    
    if (!RobotState.getRobotConfiguration().equals(RobotConfiguration.TRAVEL)){
      shooter.set(RobotState.getRobotConfiguration().shooterSpeed);
    }
    
    
    //elevator.set(RobotState.getRobotConfiguration().elevatorPos);
    if (control.getRightBumper()){
      if (!(RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_SPEAKER) )){
        shooter.set(RobotState.getChosenRobotConfiguration().shooterPos);
      }
    }
    else{
      shooter.set(RobotState.getRobotConfiguration().shooterPos);
    }

    
    //SET SHOOTER FOR REQUIRED PRESETS
    if (!(RobotState.getRobotConfiguration().equals(RobotConfiguration.INTAKE) || RobotState.getRobotConfiguration().equals(RobotConfiguration.TRAVEL))){
      
      if (control.getRightTriggerAxis() >= 0.4){
      
        shooter.setFeeder(1);
      } 
      else{
        shooter.setFeeder(0);
      }
    }
    else{
      shooter.setShooter(0);
      if (RobotState.getRobotConfiguration().equals(RobotConfiguration.TRAVEL)){
        shooter.setFeeder(0);
      }
    }

    //INTAKE PRESET
    if (RobotState.getRobotConfiguration().equals(RobotConfiguration.INTAKE)){
      //if (!shooter.getNoteDetected()){
        intake.setIntake(0.9);
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
      
      if (RobotState.getRobotConfiguration().equals(RobotConfiguration.TRAVEL)){
        shooter.setFeeder(0);
      }
      
    }

    //SPEAKER PRESET
    if (RobotState.getRobotConfiguration().equals(RobotConfiguration.SUBWOOFER) || RobotState.getRobotConfiguration().equals(RobotConfiguration.POST) || RobotState.getRobotConfiguration().equals(RobotConfiguration.FAR_POST)){ 
      //shooterLimelight.setDetectionType(DetectionType.FIDUCIAL);
      
      //Limelight tracking
      double distanceToSpeaker = Math.sqrt(Math.pow(4.5416 - shooterLimelight.botposeX, 2) + Math.pow(shooterLimelight.botposeY, 2));
      double angleToSpeaker = Math.toDegrees(Math.atan2(Constants.SPEAKER_TARGET_HEIGHT - elevator.getHeight() - Constants.ELEVATOR_MECHANISM_HEIGHT - Math.sin(shooter.getPitch() - ShooterPosition.TRAVEL.angle) * 0.75, distanceToSpeaker));
      

      //System.out.println(RobotState.getRobotConfiguration().shooterAngle.angle);
      
      shooter.tiltToAngle(RobotState.getRobotConfiguration().shooterAngle.angle);
      //System.out.println(distanceToSpeaker);
      
      /*if (shooterLimelight.isDetection && shooterLimelight.confidence() > 0.1){
        shooter.tiltToAngle(angleToSpeaker+10);
      }
      else{
        shooter.setManual(0);
      }*/

      

      VectorR direction = DriveSubsystem.getRelativeVelocity();
      direction.div(DriveSubsystem.getRelativeVelocity().getMagnitude());
      
      double adjustedAngle = shooterLimelight.x /*+ 90 - Math.toDegrees(Math.atan2(Constants.SHOOTER_VELOCITY - DriveSubsystem.getRelativeVelocity().getY(), DriveSubsystem.getRelativeVelocity().getX()))*/;

      double limelightTurnPower = MathR.limit(TURN_KP * MathR.getDistanceToAngle(0, adjustedAngle), -0.25, 0.25) * -1;
      
      //System.out.println(shooterLimelight.x);

      //if (shooterLimelight.isDetection) drive.move(leftJoystick, limelightTurnPower)
      //drive.move(leftJoystick, turnPower);
      
      
    }

    //CLIMB PRESET
    if (control.getPOV() == 90){
      elevator.setManual(0.6);
      //elevator.set(ElevatorPosition.CLIMB);
    }
    else if (control.getPOV() == 270){
      elevator.setManual(-0.6);
      //elevator.set(RobotState.getRobotConfiguration().elevatorPos);
    }
    else{
      elevator.setManual(0);
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
    if (control.getAButton()){
      intake.setIntake(-1);
    }
    /*if (control.getRightTriggerAxis() >= 0.2){
      shooter.setFeeder(1);
    }*/
    return false;
  }
}
