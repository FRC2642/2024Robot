// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.path;

import frc.robot.utils.VectorR;

/** Add your docs here. */
public class PiratePoint implements Comparable<PiratePoint> { /*The PiratePoint type is a relict of back when
     WPILib used to include vectors. It is only here to streamline the code for moving the robot from point to point*/
    
    public final VectorR position;
    public final double time;
    public double holonomicRotation;
    public final boolean stopPoint;
    

    public PiratePoint(double x, double y, double holonomicRotation, double time, Boolean stopPoint) { //This is the type corresponding to the creation of a point
        position = VectorR.fromCartesian(x, y);
        this.holonomicRotation = holonomicRotation;
        this.time = time;
        this.stopPoint = stopPoint;
    }

    public PiratePoint clone() { // This gets the point and creates a copy of it. I'm not sure entirely how it differs from just setting a new point equal to the old one, but... whatever.
        return new PiratePoint(position.getX(), position.getY(), holonomicRotation, time, stopPoint);
    }

    @Override
    public int compareTo(PiratePoint arg0) { //Compares the time.
        return Double.compare(time, arg0.time); //compare gives a positive value if time 1 > time 2, a negative if time 1 < time 2, and 0 if equal
    }

    @Override
    public String toString() { //A simple print function, entirely for the user
        return "{[pos: " + position.toString() + "ft] [t: " + VectorR.truncate(time, 2) + "s] [rot: " + VectorR.truncate(holonomicRotation, 0) + "deg]}";
    }

    //We should delete this type for 2025, it is needless and overcomplicates our existing code.
    
}
