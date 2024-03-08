// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.IPositionable;
import frc.robot.utils.MathR;

public class ShooterSubsystem extends SubsystemBase implements IPositionable<ShooterSubsystem.ShooterPosition>{

  private PIDController tiltPID = new PIDController(0.01, 0, 0);
  private PIDController shooterPID = new PIDController(0.2, 0, 0);
  private PIDController stopPID = new PIDController(0.1, 0, 0);
  
  private TalonFX shooterMotor = new TalonFX(Constants.SHOOTER_SPINNER_ID);
  private CANSparkMax shooterTiltMotor = new CANSparkMax(Constants.SHOOTER_PIVOT_ID, MotorType.kBrushless);
  private CANSparkMax feederMotor = new CANSparkMax(Constants.FEEDER_WHEELS_ID, MotorType.kBrushless);

  private static SparkAbsoluteEncoder tiltEncoder;

  private DigitalInput beamBreak = new DigitalInput(Constants.BEAM_BREAK_CHANNEL);

  private ShooterPosition currentSetPosition = ShooterPosition.TRAVEL;
  private ShooterSpeed currentSetSpeedPosition = ShooterSpeed.TRAVEL;
  private double speedLimit = 0.2;

  public static final double MAX_DEGREES = 70;
  public static final double MIN_DEGREES = -35;
  public static final int TILT_TOLERANCE = 3;
  
  public ShooterSubsystem() {
    tiltEncoder = shooterTiltMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    tiltPID.setTolerance(TILT_TOLERANCE);
    stopPID.setTolerance(10);
  }

  public double getPitch(){
    return MathR.getDistanceToAngle(0, tiltEncoder.getPosition() / (Constants.SHOOTER_TILT_ENCODER_MAX_VALUE - Constants.SHOOTER_TILT_ENCODER_MIN_VALUE) * 360 + Constants.SHOOTER_TILT_ENCODER_OFFSET, 180);
  }

  public boolean getNoteDetected(){
    return !beamBreak.get();
  }

  public void tiltToAngle(double degrees){
    shooterTiltMotor.set(MathR.limit(tiltPID.calculate(MathR.getDistanceToAngle(getPitch(), degrees), 0), -0.2, 0.2));
  }

  public void set(double speed) {
    currentSetPosition = ShooterPosition.MANUAL;

    if (ElevatorSubsystem.getPercentElevated() <= 0.4 && speed >= 0 && getPitch() <= 18){
      speed = 0;
    }

    shooterTiltMotor.set(speed);
    /*shooterTiltMotor.set(
        MathR.limitWhenReached(speed, -speedLimit, speedLimit, getPitch() <= MIN_DEGREES,
            getPitch() >= MAX_DEGREES));*/
  }

  public void set(ShooterPosition pos) {
    tiltPID.setSetpoint(0);
    double speed = MathR.limit(tiltPID.calculate(MathR.getDistanceToAngle(getPitch(), pos.angle)), -speedLimit, speedLimit);
    
    /*if (speed < 0 && pos.angle < 0 && ElevatorSubsystem.getPercentElevated() <= 0.2){
      speed = 0;
    }*/

    //System.out.println(speed);
    
    if (speed > 0 && getPitch() >= MAX_DEGREES){
      speed = 0;
    }
    else if (speed < 0 && getPitch() <= MIN_DEGREES){
      speed = 0;
    }
    
    if (!atSetPosition())
      set(speed);
    else
      set(0.0);

    currentSetPosition = pos;
  }

  public void set(ShooterSpeed speedPreset) {
    
    double motorPower = MathR.limit(tiltPID.calculate(shooterMotor.getVelocity().getValueAsDouble(), speedPreset.rpm), -1, 1);
    
    
    if (!atSetPosition())
      shooterMotor.set(motorPower);
    else
      shooterMotor.set(0);

    currentSetSpeedPosition = speedPreset;
  }

  public void setManual(double speed) {
    shooterTiltMotor.set(-speed);
  }

  public void setFeeder(double speed){
    feederMotor.set(speed);
  }

  public void setShooter(double speed){
    shooterMotor.set(-speed);
  }

  public void setShooterRPM(){
    double currentRPM = shooterMotor.getVelocity().getValueAsDouble();
    setShooter(shooterPID.calculate(currentRPM, Constants.SHOOTER_SET_RPM));
  }

  public void stopShooter(){
    double currentRPM = shooterMotor.getVelocity().getValueAsDouble();
    double speed = MathR.limit(stopPID.calculate(currentRPM, 0), -1, 1);
    
    if (Math.abs(speed) <= 0.3){
      speed = 0;
    }
    shooterMotor.set(speed);
  }


  @Override
  public boolean atSetPosition() {
    return tiltPID.atSetpoint();
  }

  @Override
  public ShooterPosition getSetPosition() {
    return currentSetPosition;
  }

  @Override
  public void setSpeedLimit(double max) {
    speedLimit = max;
  }

  @Override
  public double getSpeedLimit() {
    return speedLimit;
  }

  public void setRampRate(double rampRate) {
    shooterTiltMotor.setOpenLoopRampRate(rampRate);
  }

  public double getRampRate() {
    return shooterTiltMotor.getOpenLoopRampRate();
  }

  public enum ShooterPosition {
    TRAVEL(19),
    AMP(-20),
    TRAP(51),
    MANUAL(-1);

    public final double angle;
    private ShooterPosition(double angle) {
      this.angle = angle;
    }
  }

  public enum ShooterSpeed {
    TRAVEL(0),
    AMP(-80),
    TRAP(-120),
    SPEAKER(-200);

    public final double rpm;
    private ShooterSpeed(double rpm) {
      this.rpm = rpm;
    }
  }

  public enum ShooterAngle {
    TRAVEL(19),
    AMP(-20),
    TRAP(51),
    SUBWOOFER(60),
    POST(30),
    FAR_POST(27),
    LINE(45),
    AMP_NOTE(37),
    NOTE2(37),

    NONE(-1);

    public final double angle;
    private ShooterAngle(double angle) {
      this.angle = angle;
    }
  }

  @Override
  public void periodic() {
    //System.out.println(getNoteDetected());
    
  }
}
