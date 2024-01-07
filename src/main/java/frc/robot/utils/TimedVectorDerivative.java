// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class TimedVectorDerivative extends VectorR {
    private final Timer timer = new Timer();
    private final VectorR track;
    private final VectorR lastVector = new VectorR();

    public TimedVectorDerivative(VectorR v) {
        track = v;
        timer.start();
    }

    //formula for derivative: (dx/dt) (distance/time) (speed/time) (accell/time)
    public void update() {
        this.setFrom(track);
        this.sub(lastVector);
        this.mult(1/timer.get());
        
        timer.reset();
        lastVector.setFrom(track);
    }
}
