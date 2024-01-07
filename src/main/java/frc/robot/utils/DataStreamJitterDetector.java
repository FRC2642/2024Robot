// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/**
 * Takes a stream of numbers and returns a confidence reading based on how much
 * the set jitters.
 * This assumes that less jitter -> higher confidence
 */
public class DataStreamJitterDetector {
  private final RunningAverage averageRateOfChange = new RunningAverage(20);
  private boolean first = true;
  public static final double PERIOD_INPUT = 0.022;

  public double getConfidence() {
    return 1d / averageRateOfChange.get();
  }

  private Timer delta_t_timer = new Timer();
  public double delta_y;

  public void reset() {
    averageRateOfChange.reset();
    first = true;
    delta_t_timer.reset();
    delta_t_timer.start();
  }

  public void update(double y) {
    double delta_t = delta_t_timer.get() / PERIOD_INPUT;
    delta_y -= y;
    if (!first)
      averageRateOfChange.add(Math.abs(delta_y / delta_t));
    delta_y = y;
    delta_t_timer.reset();
    first = false;
  }

}
