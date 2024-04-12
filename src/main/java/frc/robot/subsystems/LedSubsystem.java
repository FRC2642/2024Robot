// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  /** Creates a new LedSubsystem. */

  public static Spark LEDs = new Spark(0/*PWM Channel*/);


  public LedSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void setLed(double ledNumber){
    LEDs.set(ledNumber);
  }
}
