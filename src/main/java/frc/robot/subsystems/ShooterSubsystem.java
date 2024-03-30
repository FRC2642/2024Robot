// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MathR;

public class ShooterSubsystem extends SubsystemBase{

  private PIDController tiltPID = new PIDController(0.03, 0, 0);
  private PIDController shooterPID1 = new PIDController(0.3, 0, 0);
  private PIDController shooterPID2 = new PIDController(0.2, 0, 0);
  
  private static TalonFX shooterMotor1 = new TalonFX(Constants.SHOOTER_SPINNER_ID_1);
  private static TalonFX shooterMotor2 = new TalonFX(Constants.SHOOTER_SPINNER_ID_2);
  private TalonFX shooterTiltMotor1 = new TalonFX(Constants.SHOOTER_PIVOT_ID_1);
  private TalonFX shooterTiltMotor2 = new TalonFX(Constants.SHOOTER_PIVOT_ID_2);
  private TalonFX feederMotor = new TalonFX(Constants.FEEDER_WHEELS_ID);

  private static DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(Constants.SHOOTER_ENCODER_DIGITAL_PORT_A);

  private static DigitalInput beamBreak = new DigitalInput(Constants.BEAM_BREAK_CHANNEL);

  private VelocityVoltage velocity = new VelocityVoltage(0).withUpdateFreqHz(0.0);

  public static final double TILT_TOLERANCE = 0.5;
  public static final int STOP_TOLERANCE = 20;
  public static final int SHOOTER_TOLERANCE = 2;

  public static double tiltSpeedLimit = 0.2;

  public static ArrayList<Double[]> dataArray = new ArrayList<Double[]>();

  public ShooterSubsystem() {
    tiltPID.setTolerance(TILT_TOLERANCE);
    shooterPID1.setTolerance(SHOOTER_TOLERANCE);
    shooterPID2.setTolerance(SHOOTER_TOLERANCE);

  }

  public void set(ShooterSpeed speedPreset) {

    //double motorPower1 = -MathR.limit(shooterPID1.calculate(shooterMotor1.getVelocity().getValueAsDouble()), -1, 1);
    

    double motorPower1 = (70 - shooterMotor1.getVelocity().getValueAsDouble()) * 0.03;
    double motorPower2 = MathR.limit(shooterPID2.calculate(shooterMotor2.getVelocity().getValueAsDouble(), 82), -1, 1);
    
    
    if (Math.abs(shooterMotor1.getVelocity().getValueAsDouble() - 70) <= 3){
      motorPower1 = 0;
    }
    if (shooterPID2.atSetpoint()){
      motorPower2 = 0;
    }

    shooterMotor1.setControl(new DutyCycleOut(motorPower1, true, false, false, false));
    shooterMotor2.setControl(new DutyCycleOut(-motorPower2, true, false, false, false));

    /*if (!shooterPID1.atSetpoint()){
      
      
      //shooterMotor1.setControl(velocity.withVelocity(100).withEnableFOC(true));
      //shooterMotor2.setControl(velocity.withVelocity(100).withEnableFOC(true));
      //shooterMotor1.set(motorPower1 * 0.75);
      //shooterMotor2.set(motorPower2);
    }
    else{
      shooterMotor1.set(0);
    }
    if (!shooterPID2.atSetpoint()){
      
    }
    else{
      shooterMotor2.set(0);
    }*/
  }

  public double getPitch(){
    return 180 + MathR.getDistanceToAngle(180, tiltEncoder.getAbsolutePosition() * 360 + Constants.SHOOTER_TILT_ENCODER_OFFSET);
  }

  public static double getMotorVelocity(){
    return shooterMotor2.getVelocity().getValueAsDouble();
  }

  public static boolean getNoteDetected(){
    return !beamBreak.get();
  }

  public boolean atSetSpeed(){
    return MathR.range(shooterMotor1.getVelocity().getValue(), RobotState.getRobotConfiguration().shooterSpeed.rpm, 10) && MathR.range(shooterMotor2.getVelocity().getValue(), RobotState.getRobotConfiguration().shooterSpeed.rpm, 10);
  }

  public boolean atPitch(double pitch){
    return getPitch() >= pitch - 2 && getPitch() <= pitch + 2;
  }

  public void tiltToAngle(double degrees){
    double power = MathR.limit(tiltPID.calculate(getPitch(), degrees), -tiltSpeedLimit, tiltSpeedLimit);
    
    if (Math.abs(power) <= 0.05){
      power = 0;
    }

    if (getPitch() <= 30 || getPitch() >= 355){
      power = 0;
    }
    shooterTiltMotor1.set(power);
    shooterTiltMotor2.set(-power);
  }

  public void setManual(double speed) {
    shooterTiltMotor1.set(-speed);
    shooterTiltMotor2.set(speed * 0.9);
  }

  public void setFeeder(double speed){
    feederMotor.set(speed);
  }

  public void setShooter(double speed){
    //NEGATIVE = OUT
    //POSITIVE = IN
    shooterMotor1.set(-speed * 0.75);
    shooterMotor2.set(speed);

  }

  public void stopShooter(){
    double currentRPM1 = shooterMotor1.getVelocity().getValueAsDouble();
    double speed1 = -currentRPM1 * 0.0002;

    double currentRPM2 = shooterMotor2.getVelocity().getValueAsDouble();
    double speed2 = -currentRPM2 * 0.0002;
    
    if (Math.abs(speed1) <= 0.3){
      speed1 = 0;
    }
    if (Math.abs(speed2) <= 0.3){
      speed2 = 0;
    }
    
    shooterMotor1.set(speed1);
    shooterMotor2.set(speed2);
  }


  public double getAutoAngle(double ty){
    return 0.00792889 * Math.pow(ty, 3) - 0.142877 * Math.pow(ty, 2) - 3.38697 * ty + 281.44;//0.00434251 * Math.pow(ty, 3) - 0.0437146 * Math.pow(ty, 2) - 3.81343 * ty + 278.643;
  }

  public void setSpeedLimit(double max) {
    tiltSpeedLimit = max;
  }
  

  public enum ShooterSpeed {
    TRAVEL(0),
    TRAP(-50),
    SPEAKER(-82);


    public final double rpm;
    private ShooterSpeed(double rpm) {
      this.rpm = rpm;
    }
  }

  public enum ShooterAngle {
    TRAVEL(322),
    AMP(52),
    TOP(55),
    SHOOT_ACROSS(270),
    NONE(-1);

    public final double angle;
    private ShooterAngle(double angle) {
      this.angle = angle;
    }
  }

  @Override
  public void periodic() {
    
    // System.out.println(getPitch());
    // System.out.println(shooterMotor2.getVelocity());
    // System.out.println(getPitch());
    
    
    //System.out.println(getNoteDetected());
    
  }
}
