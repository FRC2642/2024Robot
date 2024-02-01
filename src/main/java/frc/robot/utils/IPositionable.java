// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public interface IPositionable<Position> {
    public boolean atSetPosition();
    public Position getSetPosition();
    public void set(Position pos);
    public void setSpeedLimit(double max);
    public double getSpeedLimit();
    public void setRampRate(double rampRate);
    public double getRampRate();
    
}
