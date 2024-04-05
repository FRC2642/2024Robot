// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.DetectionType;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;
import frc.robot.subsystems.RobotState.RobotConfiguration;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;

public class RobotPresetCommand extends Command {
  /** Creates a new RobotPresetCommand. */
  DriveSubsystem drive;
  ShooterSubsystem shooter;
  IntakeSubsystem intake;
  LimelightSubsystem shooterLimelight;
  LimelightSubsystem intakeLimelight;
  XboxController control;
  Joystick auxButtonBoard;

  final VectorR leftJoystick = new VectorR();
  private final VectorR rightJoystick = new VectorR();
  final double TURN_KP = 0.017;
  final double LOCK_TURN_KP = 0.1;
  final double SHOOTER_LIMELIGHT_TURN_KP = 0.007;//0.0088;
  final double INTAKE_LIMELIGHT_TURN_KP = 0.018;//0.0088;
  final double GYRO_TURN_KP = 0.005555;
  private double maxSpeed = 0.25;
  private boolean isLocked = false;
  private double lockedHeading = 0;
  private boolean stopped = false;
  private boolean revUp = false;
  private Timer intakeTimer = new Timer();
  private boolean intakeTimerStarted = false;
  

  public RobotPresetCommand(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, LimelightSubsystem shooterLimelight, LimelightSubsystem intakeLimelight, XboxController control, Joystick auxButtonBoard) {
    this.drive = drive;
    this.shooter = shooter;
    this.intake = intake;
    this.shooterLimelight = shooterLimelight;
    this.intakeLimelight = intakeLimelight;
    this.control = control;
    this.auxButtonBoard = auxButtonBoard;
    addRequirements(drive, shooter, intake, shooterLimelight, intakeLimelight);
  }

  @Override
  public void initialize() {
    shooter.setSpeedLimit(0.9);
    intake.setSpeedLimit(0.5);
    shooterLimelight.setDetectionType(DetectionType.FIDUCIAL);
    intakeLimelight.setDetectionType(DetectionType.NOTE);
    RobotState.setChosenConfiguration(RobotConfiguration.SHOOT_SPEAKER);
  }

  @Override
  public void execute() {
    drive.setDefensiveMode(false);
    control.setRumble(RumbleType.kBothRumble, 0);
    
    ///////////////////////////DRIVE///////////////////////////////
    maxSpeed = MathR.lerp(0.25, 1.0, 0.0, 1.0, control.getLeftTriggerAxis());
    leftJoystick.setFromCartesian(control.getLeftX(), -control.getLeftY());
    leftJoystick.rotate(-90);
    rightJoystick.setFromCartesian(control.getRightX(), -control.getRightY());
    rightJoystick.rotate(-90);

    
    if (leftJoystick.getMagnitude() < 0.2 && rightJoystick.getMagnitude() < 0.2) {
        stopped = true;
      
        drive.stop();
        leftJoystick.setFromCartesian(0, 0);
        rightJoystick.setFromCartesian(0, 0);
      }
    else if (leftJoystick.getMagnitude() > 0.2 && rightJoystick.getMagnitude() < 0.2) {
      stopped = false;
      if (!isLocked) {
        lockedHeading = -DriveSubsystem.getYawDegrees();
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
    double turnPower;
    if (isLocked){
      turnPower = MathR.lerp(0.35, 1, 0.2, 1.0, rightJoystick.getMagnitude())  * MathR
        .limit(LOCK_TURN_KP * MathR.getDistanceToAngle(-DriveSubsystem.getYawDegrees(), angleToFace), -1, 1);
    }
    else{
      turnPower = MathR.lerp(0.35, 1, 0.2, 1.0, rightJoystick.getMagnitude())  * MathR
        .limit(TURN_KP * MathR.getDistanceToAngle(-DriveSubsystem.getYawDegrees(), angleToFace), -1, 1);
    }
      
    
    leftJoystick.mult(maxSpeed);
    
    if (!stopped && !(RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_SPEAKER) || RobotState.getRobotConfiguration().equals(RobotConfiguration.INTAKE))){
      drive.move(leftJoystick, turnPower * maxSpeed);
    }
    
    
    
    
    ////////////////////////////////////////////////////////


    
    //Set subsystem presets
    intake.set(RobotState.getRobotConfiguration().intakePos);
    
    
    if (!(RobotState.getRobotConfiguration().equals(RobotConfiguration.TRAVEL) || RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_AMP) || RobotState.getRobotConfiguration().equals(RobotConfiguration.TRAP)) && !auxButtonBoard.getRawButton(11)){
      shooter.set(RobotState.getRobotConfiguration().shooterSpeed);
    }

    if (RobotState.getRobotConfiguration().equals(RobotConfiguration.TRAP)){
      shooter.setTrapSpeed(10);
      shooter.setJet();
    }
    else{
      shooter.stopJet();
    }
    if (auxButtonBoard.getRawButton(9)){
      revUp = true;
      shooter.set(RobotState.getShooterSpeed());
    }
    else{
      revUp = false;
    }
    
    

    //Set shooter angle for intake and amp, auto angle shooter for everything else
    if (RobotState.getRobotConfiguration().equals(RobotConfiguration.INTAKE) || RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_AMP) || RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_OVER) || RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_ACROSS) || RobotState.getRobotConfiguration().equals(RobotConfiguration.TRAP) || (RobotState.getRobotConfiguration().equals(RobotConfiguration.TRAVEL) && !RobotState.getChosenRobotConfiguration().equals(RobotConfiguration.SHOOT_SPEAKER))){
      shooter.tiltToAngleAMP(RobotState.getRobotConfiguration().shooterAngle.angle);
      
    }
    else if (shooterLimelight.isDetection){
      shooter.tiltToAngle(shooter.getAutoAngle(shooterLimelight.y, shooterLimelight.a));
    }
    else if (RobotState.getChosenRobotConfiguration().equals(RobotConfiguration.SHOOT_AMP) || RobotState.getChosenRobotConfiguration().equals(RobotConfiguration.SHOOT_OVER)){
      shooter.tiltToAngle(RobotState.getRobotConfiguration().shooterAngle.angle);
    }
    else{
      shooter.setManual(0);
    }
    
    //SET SHOOTER FOR REQUIRED PRESETS
    if (RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_SPEAKER) || RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_OVER) || RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_ACROSS) || RobotState.getRobotConfiguration().equals(RobotConfiguration.TRAP)){
      if (control.getRightTriggerAxis() >= 0.4) 
        shooter.setFeeder(1);
      
      else
        shooter.setFeeder(0);
      
    }
    else if (RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_AMP)){
      if (control.getRightTriggerAxis() >= 0.4) 
        shooter.setFeeder(-1);
      
      else
        shooter.setFeeder(0);
    }
    else{
      if (!revUp){
        
        shooter.setShooter(0);
      }
      
      if (RobotState.getRobotConfiguration().equals(RobotConfiguration.TRAVEL)){
        shooter.setFeeder(0);
      }
    }

    //INTAKE PRESET
    if (RobotState.getRobotConfiguration().equals(RobotConfiguration.INTAKE)){
      
        shooter.stopShooter();
        intake.setIntake(1);
        if (!(ShooterSubsystem.getNoteDetected() || ShooterSubsystem.getCloseNoteDetected())){
          shooter.setFeeder(0.9);
          intakeTimerStarted = false;
        } 
        else if (ShooterSubsystem.getCloseNoteDetected() && !ShooterSubsystem.getNoteDetected()){
          shooter.setFeeder(0.2);
        }
        else if (ShooterSubsystem.getNoteDetected()){
          intake.set(IntakePosition.RETRACTED);
          control.setRumble(RumbleType.kBothRumble, 1);
          shooter.setFeeder(0);
        }
        

        double limelightTurnPower = MathR.limit(INTAKE_LIMELIGHT_TURN_KP * intakeLimelight.x, -0.20, 0.20) * -1;

        if (Math.abs(MathR.getDistanceToAngle(0, intakeLimelight.x)) <= 2){
          limelightTurnPower = 0;
        }
        if (intakeLimelight.isDetection && intakeLimelight.confidence() >= 0.2 && IntakeSubsystem.getPitch() <= 10){
          
          drive.move(leftJoystick, limelightTurnPower);
        }
        else{
          drive.move(leftJoystick, turnPower * maxSpeed);
        }
        
        
        
      
      
      
    }
    else if (!(RobotState.getRobotConfiguration().equals(RobotConfiguration.INTAKE))){
      intake.setIntake(0);
      
      if (RobotState.getRobotConfiguration().equals(RobotConfiguration.TRAVEL)){
        shooter.setFeeder(0);
      }
      
    }


    //SPEAKER PRESET
    if (RobotState.getRobotConfiguration().equals(RobotConfiguration.SHOOT_SPEAKER)){ 
      //shooter.tiltToAngle(shooter.getAutoAngle(shooterLimelight.y));
      //shooter.tiltToAngle(RobotContainer.ANGLE);


      double leadAngle = Math.abs(Math.toDegrees(Math.atan2(Constants.SHOOTER_VELOCITY - DriveSubsystem.getRelativeVelocity().getY(), DriveSubsystem.getRelativeVelocity().getX() + 0.000001))) - 90;
      double deltaAngle =  -shooter.getAutoOffset(shooterLimelight.y) + shooterLimelight.x;

      /*if (DriveSubsystem.getRelativeVelocity().getX() > 0){
        adjustedAngle = shooterLimelight.x + leadAngle;
      }
      else if (DriveSubsystem.getRelativeVelocity().getX() < 0){
        adjustedAngle = shooterLimelight.x - leadAngle;
      }*/
       

      double limelightTurnPower = MathR.limit(SHOOTER_LIMELIGHT_TURN_KP * deltaAngle, -0.11, 0.11) * -1.8;
      
      //System.out.println(limelightTurnPower + " " + MathR.getDistanceToAngle(0, adjustedAngle));
      
      System.out.println(shooter.getAutoOffset(shooterLimelight.y));
      System.out.println("delta: "+deltaAngle);
      System.out.println("turn power: "+limelightTurnPower);
      
      if (Math.abs(limelightTurnPower) <= 0.02){
        limelightTurnPower = 0;
      }
      
      
      if (shooterLimelight.isDetection && !(deltaAngle >= -0.5 && deltaAngle <= 0.5)) {
        drive.move(leftJoystick, limelightTurnPower);
      }
      else if (shooterLimelight.isDetection && (deltaAngle >= -0.5 && deltaAngle <= 0.5) && leftJoystick.getMagnitude() <= 0.2 && rightJoystick.getMagnitude() <= 0.2){
        drive.stop();
      }
      else{
        drive.move(leftJoystick, turnPower);
      }  
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (control.getYButton()){
      shooter.setShooter(0.3);
    }
    if (control.getXButton()){
      shooter.setFeeder(-0.4);
    }
    if (control.getBButton()){
      intake.setIntake(-1);
    }
    if (control.getAButton()){
      shooter.setFeeder(0.4);
    }
    if (auxButtonBoard.getRawButton(7)){
      shooter.setJet();
    }
    
    
    
    return false;
  }
}
