// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  /** Creates a new LedSubsystem. */

  public static Spark LEDs = new Spark(1/*PWM Channel*/);


  public LedSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void setLed(double ledNumber){
    
    
    LEDs.set(ledNumber);
  }

  public static void setColor(LEDColor color){
    LEDs.set(color.num);
  }

  public enum LEDColor{
    YELLOW(0.69),
    PURPLE(0.91),
    PINK(0.57),
    GREEN(0.77),
    BLUE(0.85);

    public final double num;
    private LEDColor(double num){
      this.num = num;
    }
  }
}