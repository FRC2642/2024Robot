// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MathR;

public class ShooterSubsystem extends SubsystemBase{

  private PIDController tiltPID = new PIDController(0.01, 0, 0);
  private PIDController shooterPID = new PIDController(0.2, 0, 0);
  private PIDController stopPID = new PIDController(0.18, 0, 0);
  
  private TalonFX shooterMotor = new TalonFX(Constants.SHOOTER_SPINNER_ID);
  private CANSparkMax shooterTiltMotor = new CANSparkMax(Constants.SHOOTER_PIVOT_ID, MotorType.kBrushless);
  private CANSparkMax feederMotor = new CANSparkMax(Constants.FEEDER_WHEELS_ID, MotorType.kBrushless);

  private static SparkAbsoluteEncoder tiltEncoder;

  private DigitalInput beamBreak = new DigitalInput(Constants.BEAM_BREAK_CHANNEL);


  public static final double MAX_DEGREES = 70;
  public static final double MIN_DEGREES = -35;
  public static final int TILT_TOLERANCE = 2;
  public static final int STOP_TOLERANCE = 20;
  public static final int SHOOTER_TOLERANCE = 10;

  public static double tiltSpeedLimit = 0.2;

  public ShooterSubsystem() {
    tiltEncoder = shooterTiltMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    tiltPID.setTolerance(TILT_TOLERANCE);
    stopPID.setTolerance(STOP_TOLERANCE);
    shooterPID.setTolerance(SHOOTER_TOLERANCE);
  }

  public void set(ShooterSpeed speedPreset) {
    double motorPower = MathR.limit(shooterPID.calculate(shooterMotor.getVelocity().getValueAsDouble(), speedPreset.rpm), -1, 1);
    
    if (!shooterPID.atSetpoint())
      shooterMotor.set(motorPower);
    else
      shooterMotor.set(0);
  }

  public double getPitch(){
    return MathR.getDistanceToAngle(0, tiltEncoder.getPosition() / (Constants.SHOOTER_TILT_ENCODER_MAX_VALUE - Constants.SHOOTER_TILT_ENCODER_MIN_VALUE) * 360 + Constants.SHOOTER_TILT_ENCODER_OFFSET, 180);
  }

  public boolean getNoteDetected(){
    return !beamBreak.get();
  }

  public void tiltToAngle(double degrees){
    shooterTiltMotor.set(MathR.limit(tiltPID.calculate(MathR.getDistanceToAngle(getPitch(), degrees), 0), -tiltSpeedLimit, tiltSpeedLimit));
  }

  public void setManual(double speed) {
    shooterTiltMotor.set(-speed);
  }

  public void setFeeder(double speed){
    feederMotor.set(speed);
  }

  public void setShooter(double speed){
    //NEGATIVE = OUT
    //POSITIVE = IN
    shooterMotor.set(-speed);
  }

  public void stopShooter(){
    double currentRPM = shooterMotor.getVelocity().getValueAsDouble();
    double speed = MathR.limit(stopPID.calculate(currentRPM, 0), -1, 1);
    
    if (Math.abs(speed) <= 0.2){
      speed = 0;
    }
    shooterMotor.set(speed);
  }


  public double getAutoAngle(double ty){
    return 0.7877829265 * ty + 47.6096773089;
  }

  public void setSpeedLimit(double max) {
    tiltSpeedLimit = max;
  }
  

  public enum ShooterSpeed {
    TRAVEL(0),
    AMP(-80),
    TRAP(-50),
    SPEAKER(-200);

    public final double rpm;
    private ShooterSpeed(double rpm) {
      this.rpm = rpm;
    }
  }

  public enum ShooterAngle {
    TRAVEL(19),
    AMP(-20),

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
