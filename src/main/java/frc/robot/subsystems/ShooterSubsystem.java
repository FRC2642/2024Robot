// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.MathR;

public class ShooterSubsystem extends SubsystemBase{

  private PIDController tiltPID = new PIDController(0.06, 0, 0);
  private PIDController shooterPID1 = new PIDController(0.3, 0, 0);
  private PIDController shooterPID2 = new PIDController(0.2, 0, 0);

  PIDController motorPID1 = new PIDController(0.01, 0, 0);
  PIDController motorPID2 = new PIDController(0.01, 0, 0);
  
  private static TalonFX shooterMotor1 = new TalonFX(Constants.SHOOTER_SPINNER_ID_1);
  private static TalonFX shooterMotor2 = new TalonFX(Constants.SHOOTER_SPINNER_ID_2);
  private TalonFX shooterTiltMotor1 = new TalonFX(Constants.SHOOTER_PIVOT_ID_1);
  private TalonFX shooterTiltMotor2 = new TalonFX(Constants.SHOOTER_PIVOT_ID_2);
  private TalonFX feederMotor = new TalonFX(Constants.FEEDER_WHEELS_ID);

  private static DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(Constants.SHOOTER_ENCODER_DIGITAL_PORT_A);

  private static DigitalInput farBeamBreak = new DigitalInput(Constants.FAR_BEAM_BREAK_CHANNEL);
  private static DigitalInput closeBeamBreak = new DigitalInput(Constants.CLOSE_BEAM_BREAK_CHANNEL);

  private static PWM jet = new PWM(0);
  private VelocityVoltage velocity = new VelocityVoltage(0).withUpdateFreqHz(0.0);

  public static final double TILT_TOLERANCE = 0.15;
  public static final int STOP_TOLERANCE = 20;
  public static final int SHOOTER_TOLERANCE = 2;

  public static double tiltSpeedLimit = 0.2;

  public static ArrayList<Double[]> dataArray = new ArrayList<Double[]>();

  private final Slot0Configs controllerConfig = new Slot0Configs();
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  public ShooterSubsystem() {
    //Set shooter tilt tolerance
    tiltPID.setTolerance(TILT_TOLERANCE);
    // shooterPID1.setTolerance(SHOOTER_TOLERANCE);
    // shooterPID2.setTolerance(SHOOTER_TOLERANCE);

    //Create config and limit the current supply
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    //Set pid and feedforward configs
    controllerConfig.kP = 0.25;
    controllerConfig.kI = 0;
    controllerConfig.kD = 0.012;
    controllerConfig.kS = 0.8;
    controllerConfig.kV = 0.00105;
    controllerConfig.kA = 0;

    //Apply configs
    shooterMotor1.getConfigurator().apply(config, 1.0);
    shooterMotor2.getConfigurator().apply(config, 1.0);
    shooterMotor1.getConfigurator().apply(controllerConfig, 1.0);
    shooterMotor2.getConfigurator().apply(controllerConfig, 1.0);
  }

  //Run shooter wheels
  public void runVelocity(double leftRpm, double rightRpm, double leftFeedforward, double rightFeedforward) {
    shooterMotor1.setControl(velocityControl.withVelocity(leftRpm / 60.0).withFeedForward(leftFeedforward));
    shooterMotor2.setControl(velocityControl.withVelocity(-rightRpm / 60.0).withFeedForward(rightFeedforward));
  }

  /*
  public void set(ShooterSpeed speedPreset) {

    //double motorPower1 = -MathR.limit(shooterPID1.calculate(shooterMotor1.getVelocity().getValueAsDouble()), -1, 1);
    

    double motorPower1 = (70 - shooterMotor1.getVelocity().getValueAsDouble()) * 0.03;
    double motorPower2 = MathR.limit(shooterPID2.calculate(shooterMotor2.getVelocity().getValueAsDouble(), 82), -1, 1);
    
    System.out.println(shooterMotor1.getVelocity().getValueAsDouble());
    
    
    if (Math.abs(shooterMotor1.getVelocity().getValueAsDouble() - 70) <= 3){
      motorPower1 = 0;
    }
    if (shooterPID2.atSetpoint()){
      motorPower2 = 0;
    }
    VelocityDutyCycle motor1Velocity = new VelocityDutyCycle(20);
    VelocityDutyCycle motor2Velocity = new VelocityDutyCycle(10);

    shooterMotor1.setControl(motor1Velocity.withEnableFOC(true));
    shooterMotor2.setControl(motor2Velocity.withEnableFOC(true));

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
    }
  }*/

  //Get shooter pitch angle
  public double getPitch(){
    return 180 + MathR.getDistanceToAngle(180, tiltEncoder.getAbsolutePosition() * 360 + Constants.SHOOTER_TILT_ENCODER_OFFSET);
  }

  //Get status of the exit beam break
  public static boolean getNoteDetected(){
    return !farBeamBreak.get();
  }
  //Get status of the enter beam break
  public static boolean getCloseNoteDetected(){
    return !closeBeamBreak.get();
  }

  //Check if the shooter is at a certain pitch within a tolerance
  public boolean atPitch(double pitch){
    return getPitch() >= pitch - 1 && getPitch() <= pitch + 1;
  }

  //Tilt the shooter to a certain angle while avoiding going back into the intake
  public void tiltToAngle(double degrees){
    double power = MathR.limit(tiltPID.calculate(MathR.getDistanceToAngle(getPitch(), degrees, 10), 0), -tiltSpeedLimit, tiltSpeedLimit);
    
    if (Math.abs(power) <= 0.01){
      power = 0;
    }

    if ((getPitch() <= 30  && power < 0) || (getPitch() >= 200 && power > 0)){
      power = 0;
    }


    //shooterMotor1.setControl(new DutyCycleOut(power, true, false, false, false));
    //shooterMotor2.setControl(new DutyCycleOut(power, true, false, false, false));
     shooterTiltMotor1.set(power);
     shooterTiltMotor2.set(-power);
  }

  //Run the motors
  public void setManual(double speed) {
    //System.out.println(speed);
    
    shooterTiltMotor1.set(-speed);
    shooterTiltMotor2.set(speed * 0.9);
  }

  //Activate the jet
  public void setJet(){
    jet.setSpeed(1);
  }
  //Stop the jet
  public void stopJet(){
    jet.setSpeed(0);
  }

  //Set the feeder wheels
  public void setFeeder(double speed){
    feederMotor.setControl(new DutyCycleOut(speed, true, false, false, false));
    //feederMotor.set(speed);
  }

  //Set the shooter to a certain soeed
  public void setShooter(double speed){
    //NEGATIVE = OUT
    //POSITIVE = IN
    shooterMotor1.set(-speed * 0.75);
    shooterMotor2.set(speed);

  }
  //Use a proportion to stop the shooter immediately
  public void stopShooter(){
    double currentRPM1 = shooterMotor1.getVelocity().getValueAsDouble();
    double speed1 = -currentRPM1 * 0.003;

    double currentRPM2 = shooterMotor2.getVelocity().getValueAsDouble();
    double speed2 = -currentRPM2 * 0.003;
    
    if (Math.abs(speed1) <= 0.4){
      speed1 = 0;
    }
    if (Math.abs(speed2) <= 0.4){
      speed2 = 0;
    }
    
    shooterMotor1.set(speed1);
    shooterMotor2.set(speed2);
  }

  //Find the angle that the shooter needs to be angled to the speaker based on limelight data
  public double getAutoAngle(double ty, double ta){
    double angle = 0.0000178887 * Math.pow(ty, 5) - 0.000522387 * Math.pow(ty, 4) + 0.00114773 * Math.pow(ty, 3) + 0.0493169 * Math.pow(ty, 2) + 1.55427 * ty + 57.8461;
    double gyroDistToZero = Math.abs(MathR.limit(MathR.getDistanceToAngle(DriveSubsystem.getYawDegrees(), 0), -10, 10)) * 0.02;

    return angle - gyroDistToZero + RobotContainer.OFFSET;

    /*if (ta <= 0.5){
      return -0.00000714563 * Math.pow(ty, 5) + 0.000248267 * Math.pow(ty, 4) + 0.00716947 * Math.pow(ty, 3) - 0.184703 * Math.pow(ty, 2) - 3.07547 * ty + 281.267 - RobotContainer.OFFSET;
    }
    else{
      return -0.00000714563 * Math.pow(ty, 5) + 0.000248267 * Math.pow(ty, 4) + 0.00716947 * Math.pow(ty, 3) - 0.184703 * Math.pow(ty, 2) - 3.07547 * ty + 281.267 - RobotContainer.OFFSET;
    }*/
    //Old formula
    //0.00000276475 * Math.pow(ty, 5) - 0.0000942192 * Math.pow(ty, 4) - 0.0000239911 * Math.pow(ty, 3) + 0.0202433 * Math.pow(ty, 2) + 1.59285 * ty + 58.4301;
    
  }

  //Find the angle that the robot needs to turn to be angled to the april tag based on limelight data
  public double getAutoOffset(double ty){
    return -0.00336373 * Math.pow(ty, 2) - 0.212577 * ty + 5.31239 - 7;
  }

  //Set shooter tilt speed limits
  public void setSpeedLimit(double max) {
    tiltSpeedLimit = max;
  }
  
  //Shooter speed names
  public enum ShooterSpeed {
    /*TRAVEL(0),
    IDLE(-30),
    SPEAKER(-82);*/

    TRAVEL(0),
    PASS(3200),
    TRAP(2500),
    SPEAKER(6000);


    public final double rpm;
    private ShooterSpeed(double rpm) {
      this.rpm = rpm;
    }
  }

  //Shooter angle names
  public enum ShooterAngle {
    TRAVEL(40),
    AMP(176),
    TOP(180),
    TRAP(150/*205*/),
    PASS(70),
    SUBWOOFER(100),
    CLIMB_UP(190),
    CLIMB_DOWN(85),
    NONE(-1);

    public final double angle;
    private ShooterAngle(double angle) {
      this.angle = angle;
    }
  }

  @Override
  public void periodic() {
    
    //System.out.println(getPitch());
  }
}
