// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.PrintCommand;

/** Wrapper class for vectors */
public class VectorR implements Cloneable {

    // vector data
    private double x, y;

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getAngle() {
        return Math.toDegrees(Math.atan2(y, x));
    }

    public double getMagnitude() {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public VectorR() {
        x = 0;
        y = 0;
    }

    public void add(VectorR... vector) {
        for (var vec : vector) {
            x += vec.x;
            y += vec.y;
        }
    }

    public void sub(VectorR vector) {
        x -= vector.x;
        y -= vector.y;
    }

    public void pow(double val) {
        setMagnitude(Math.pow(getMagnitude(), val));
    }

    public void mult(double val) {
        setMagnitude(getMagnitude() * val);
    }

    public void div(double val){
        setMagnitude(getMagnitude() / val);
    }

    public double dot(VectorR v2) {
        return v2.getX() * x + v2.getY() * y;
    }

    public static VectorR addVectors(VectorR... vectors) {
        VectorR v3 = new VectorR();
        
        for (VectorR vector : vectors){
            v3.add(vector);
        }
        return v3;
    }

    /*
     * Subtracts the following vectors from the first vector
     */
    public static VectorR subVectors(VectorR... vectors) {
        VectorR v3 = vectors[0].clone();
        
        for (int i = 1; i < vectors.length; i++){
            v3.sub(vectors[i]);
        }
        return v3;
    }

    public void setFromPolar(double magnitude, double angleDegrees) {
        x = magnitude * Math.cos(Math.toRadians(angleDegrees));
        y = magnitude * Math.sin(Math.toRadians(angleDegrees));
    }

    public void setFromCartesian(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setMagnitude(double mag) {
        setFromPolar(mag, getAngle());
    }

    public void setAngle(double degrees) {
        setFromPolar(getMagnitude(), degrees);
    }

    public void setX(double val) {
        x = val;
    }

    public void setY(double val) {
        y = val;
    }

    public void setFrom(VectorR v) {
        x = v.x;
        y = v.y;
    }

    public double getTerminalMagnitude() {
        return getMagnitude() * -1;
    }

    public double getTerminalAngle() {
        return getAngle() + 180;
    }


    public static VectorR fromPolar(double magnitude, double angleDegrees) {
        VectorR v = new VectorR();
        v.setFromPolar(magnitude, angleDegrees);
        return v;
    }

    public static VectorR fromCartesian(double x, double y) {
        VectorR v = new VectorR();
        v.setFromCartesian(x, y);
        return v;
    }

    public void rotate(double angle) {
        setAngle(getAngle() + angle);
    }
    
    @Override
    public VectorR clone() {
        return fromCartesian(x, y);
    }

    @Override
    public String toString() {
        return "<" + truncate(x,2) + "," + truncate(y,2) + ">";
    }

    public boolean compare(VectorR v, double distanceThreshold, double angleDegThreshold) {
        boolean magnitude = Math.abs(getMagnitude() - v.getMagnitude()) < distanceThreshold;
        boolean angle = Math.abs(MathR.getDistanceToAngle(getAngle(), v.getAngle())) < angleDegThreshold;
        return magnitude && angle;
    }

    public static String truncate(double value, int decimals) {
        if (value % 1 == 0) return Double.toString(value);
        String val = Double.toString(value);
        String[] split = val.split("\\.");
        if (decimals == 0) return split[0];
        return split[0] + "." + split[1].substring(0, decimals);
    }
}