// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.swerve.SwerveModules.ModuleLocation;

import frc.robot.subsystems.swerve.SwerveModuleInfo;

public final class Constants {

    // Ratios
    public static final double FEET_PER_DISPLACEMENT = 0.1248804253858982; //CHANGE
    public static final double MODULE_ANGLE_KP = 0.00524;
    public static final double FOOT_PER_METER = 3.28084;

    // Controllers
    public static final int DRIVE_CONTROL_PORT = 0;
    public static final int AUX_BUTTON_BOARD_PORT = 1;

    // Swerve
    public static final SwerveModuleInfo FRONT_RIGHT = new SwerveModuleInfo(1, 2, 11, 0 /*CHANGE*/, 1, -1, ModuleLocation.FRONT_RIGHT);
    public static final SwerveModuleInfo FRONT_LEFT = new SwerveModuleInfo(3, 4, 12, 0 /*CHANGE*/, 1, 1, ModuleLocation.FRONT_LEFT);
    public static final SwerveModuleInfo BACK_RIGHT = new SwerveModuleInfo(5, 6, 13, 0 /*CHANGE*/, -1, -1, ModuleLocation.BACK_RIGHT);
    public static final SwerveModuleInfo BACK_LEFT = new SwerveModuleInfo(7, 8, 14, 0 /*CHANGE*/, -1, 1, ModuleLocation.BACK_LEFT);

    // Shooter
    public static final int SHOOTER_SPINNER_ID = 21;
    public static final int SHOOTER_PIVOT_ID = 22;
    public static final int FEEDER_WHEELS_ID = 23;

    // Elevator
    public static final int ELEVATOR_MOTOR_1_ID = 31;
    public static final int ELEVATOR_MOTOR_2_ID = 32;
    public static final int ELEVATOR_ENCODER_DIGITAL_PORT_A = 0;
    public static final int ELEVATOR_ENCODER_DIGITAL_PORT_B = 1;

    // Intake
    public static final int INTAKE_PIVOT_ID = 41;
    public static final int INTAKE_SPINNER_ID = 42;

    // Field Dimensions
    public static final double FIELD_X = 54d + 1d/12d;
    public static final double FIELD_Y = 26d + 7d/12d;

    // Various Stats
    public static final double SHOOTER_VELOCITY = 7; //NOTE SHOOT FEET PER SECOND //CHANGE
    public static final double SHOOTER_SET_RPM = 4000; // CHANGE
    public static final double SPEAKER_TARGET_HEIGHT = 7.8333;
    public static final double SHOOTER_TILT_ENCODER_MAX_VALUE = 0; //CHANGE
    public static final double SHOOTER_TILT_ENCODER_MIN_VALUE = 0; //CHANGE
    public static final double SHOOTER_TILT_ENCODER_OFFSET = 0; //CHANGE
    
    public static final double ELEVATOR_ENCODER_MAX_VALUE = 0; //CHANGE
    public static final double ELEVATOR_ENCODER_OFFSET = 0; //CHANGE
    public static final double ELEVATOR_MAX_HEIGHT_FEET = 0; //CHANGE
    public static final double ELEVATOR_MECHANISM_HEIGHT = 0.39583333;

    public static final double INTAKE_TILT_ENCODER_MAX_VALUE = 0; //CHANGE
    public static final double INTAKE_TILT_ENCODER_MIN_VALUE = 0; //CHANGE
    public static final double INTAKE_TILT_ENCODER_OFFSET = 0; //CHANGE

    public static final int BEAM_BREAK_CHANNEL = 0; //CHANGE

}
