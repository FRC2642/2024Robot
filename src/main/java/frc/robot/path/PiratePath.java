// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.path;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.TreeSet;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.utils.Easings;
import frc.robot.utils.MathR;

/**
 * Represents a field path for a swerve robot, contains helper functions for
 * reading and building paths. CANNOT BE CHANGED WHILE READING
 */
public class PiratePath extends TreeSet<PiratePoint> {

    public static final ObjectMapper JSON_MAPPER = new ObjectMapper();
    public static final PiratePoint DEFAULT_VALUE = new PiratePoint(0, 0, 0, 0, false);
    public static final String PARENT_DIRECTORY = Filesystem.getDeployDirectory().getAbsolutePath()
            + "/pathplanner/generatedJSON/";

    public final boolean allianceDependent;
    public String name;
    public boolean choreo;

    /*
     * Creates an empty path
     */
    public PiratePath(boolean allianceDependent) {
        this.allianceDependent = allianceDependent;
        name = "unnamed";
    }

    /*
     * Creates a path from a JSON file from FRC PathPlanner 2023
     */
    public PiratePath(String name, boolean choreo) {
        this.name = name;
        this.choreo = choreo; //This doesn't matter anymore
        Exception e;
        if (choreo){ //Nobody likes or uses choreo. This code is useless.
            e = trySetFromPathPlannerJSON(new File(PARENT_DIRECTORY, name + ".json"));
        }
        else{ //Reads the path and stores each point. "e" just stores the error code that it hands back, if it hands one back.
            e = trySetFromPathPlannerJSON(new File(PARENT_DIRECTORY, name + ".wpilib.json"));
        }
        if (e != null) { //If there's an error code, tell the user.
            e.printStackTrace();
            add(DEFAULT_VALUE);
        }
        allianceDependent = true;
    }

    public Exception trySetFromPathPlannerJSON(File jsonFile) {
       
        try {
            JsonNode root = JSON_MAPPER.readTree(jsonFile); //Creates a tree of values from the generated JSON from earlier. Imagine a list, but it branches.
            var pointIterator = root.elements();
            boolean first = true; //We start with the first point of the path, where the robot stops.

            //A couple definitions, explained below
            double t = 0.0;
            double x = 0.0;
            double y = 0.0;
            double r = 0.0;
            boolean stop = false;
            
            while (pointIterator.hasNext()) {
                var point = pointIterator.next();
                
                if (choreo){ //Literally does not matter. We'll delete this later.
                    t = point.get("timestamp").asDouble();
                    x = (Constants.FIELD_X) - (point.get("x").asDouble() * Constants.FOOT_PER_METER);
                    y = (Constants.FIELD_Y) - (point.get("y").asDouble() * Constants.FOOT_PER_METER);
                    r = point.get("heading").asDouble() + 180;
                    stop = point.get("velocityX").asDouble() == 0.0 && point.get("velocityY").asDouble() == 0.0 && !first;
                }
                else{
                    JsonNode pose = point.get("pose"); //Get pose and translation, which are two branches containing the rest of the stuff in the while loop below
                    JsonNode translation = pose.get("translation");

                    t = point.get("time").asDouble();
                    x = /*(Constants.FIELD_X) -*/ (translation.get("x").asDouble() * Constants.FOOT_PER_METER);
                    y = /*(Constants.FIELD_Y) -*/ (translation.get("y").asDouble() * Constants.FOOT_PER_METER);
                    r = point.get("holonomicRotation").asDouble();
                    stop = point.get("velocity").asDouble() == 0.0 && !first;
                }

                PiratePoint pt = new PiratePoint(x, y, r, t, stop); //Creates a point from the above information; see PiratePoint for specifics
                add(pt);
                first = false; //We are, after the while loop runs for the first time, no longer on the first point of the path.
            }
        } catch (Exception e) {
            return e; //If this doesn't work for whatever reason, we hand back the error code.
        }
        return null; //Otherwise, we don't had back an error code.
    }

    public PiratePath getRedAlliance() {
        PiratePath redPath = new PiratePath(true);
        redPath.name = "RED " + this.name;

        for (var pt : this) { //TODO figure out how this works
            double Y = pt.position.getY();
            var newPt = pt.clone();
            newPt.position.setY(Math.abs(Constants.FIELD_Y - Y));
            newPt.holonomicRotation = -newPt.holonomicRotation;
            redPath.add(newPt);
        }

        return redPath;
    }

    public ArrayList<PiratePath> getSubPaths() { //I have no idea why there are two getSubPaths functions; probably for different arg lengths. I didn't make this okay?
        ArrayList<PiratePath> paths = new ArrayList<>(); //Makes an array to store the mutliple paths (?)

        PiratePath current = new PiratePath(true); //Reads the path, as explained above
        int index = 0;
        for (var pt : this) { //For those who don't use java, this is "for each point in the array"
            current.add(pt); //Add it to the path (huh?)
            
            if (pt.stopPoint && pt.time != 0.0) { //If it's the last point
                current.name = this.name + "[" + index + "]";
                paths.add(current);
                current = new PiratePath(true);
                index++;
            } //Okay I've got nothing. TODO
        }
        return paths;
    }

    public ArrayList<PiratePath> getSubPaths(ArrayList<Double> separatePathAtTimes, double rangeToSeparate) {
        ArrayList<PiratePath> paths = new ArrayList<>();
        HashMap<Double, Double> separatedAtTime = new HashMap<>(separatePathAtTimes.size());

        PiratePath current = new PiratePath(true); //Creates a new path, like the other one
        int index = 0;
        if (separatePathAtTimes.isEmpty()){
            return getSubPaths(); //TF man. TODO what is this?
        }
        else{
            for (var pt : this) {
                current.add(pt);
                for (Double time : separatePathAtTimes){
                    if (pt.stopPoint || MathR.range(time, pt.time, rangeToSeparate) && pt.time != 0.0) { //If it's a stop point OR just look at mathr to decipher that part AND it isnt the first point (?)
                        if (!separatedAtTime.containsKey(time) && MathR.range(separatedAtTime.get(time), pt.time, rangeToSeparate)){ //I'm sorry, WHY are there more conditions? TODO And what do they mean?
                            separatedAtTime.put(time, pt.time);
                            current.name = this.name + "[" + index + "]";
                            paths.add(current);
                            current = new PiratePath(true);
                            index++;
                        }
                    }
                }
            }
        }
        return paths;
    }

    

    @Override
    public String toString() { //Turns a path into a printable string of points
        ArrayList<String> s = new ArrayList<>();
        for (var pt : this) {
            s.add(pt.toString());
        }
        return name + "\n" + String.join("\n", s);
    }

    public void print() { //Prints whatever you want it to print, but *fancy*
        System.out.println("--------------------------------------------------------------------------------------------------------------");
        System.out.println(toString());
        System.out.println("--------------------------------------------------------------------------------------------------------------");
    }
    
    public static void print(ArrayList<PiratePath> paths) { //Prints out everything in the list, using the function above.
        for (var p : paths) {
            p.print();
        } 
    }
    
    public static void print(ArrayList<PiratePath> paths, int index) { //Seriously. How much debug do we need here? Prints the index.
        paths.get(index).print();
    }

    public void fillWithSubPointsEasing(double timeBetweenPts, Easings.Functions interpolationEasing) { //Interpolates between points. Basically, smoothes out the stuff between points so the robot isn't jerky

        ArrayList<PiratePoint> points = new ArrayList<>(this);

        for (int i = 0; i < points.size() - 1; i++) { //For each pair of consecutive points
            PiratePoint current = points.get(i);
            PiratePoint next = points.get(i + 1);

            for (double time = current.time; time < next.time; time += timeBetweenPts) { //Creates a bunch of in-between points with interpolated, average-ish values. Check out easings in utils for more :) (this is dumb)
                double x = Easings.interpolate(current.position.getX(), next.position.getX(), current.time, next.time,
                        time, interpolationEasing);
                double y = Easings.interpolate(current.position.getY(), next.position.getY(), current.time, next.time,
                        time, interpolationEasing);
                double h = Easings.interpolate(current.holonomicRotation, next.holonomicRotation, current.time, next.time, time,
                        interpolationEasing);
                add(new PiratePoint(x, y, h, time, false));
            }
        }
    }

    public double getLastTime() { //Gets the last time
        return last().time;
    }

    public PiratePoint getFirst() { //Gets the first point
        return first();
    }
}

//Lots to be done here next year.