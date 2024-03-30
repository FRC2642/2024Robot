// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.swerve.SwerveModuleInfo;
import frc.robot.subsystems.swerve.SwerveModules.ModuleLocation;

public final class Constants {

    // Ratios
    public static final double FEET_PER_DISPLACEMENT = 0.15462531;
    public static final double MODULE_ANGLE_KP = -0.00524;
    public static final double FOOT_PER_METER = 3.28084;

    // Controllers
    public static final int DRIVE_CONTROL_PORT = 0;
    public static final int AUX_BUTTON_BOARD_PORT = 1;

    // Swerve
    public static final SwerveModuleInfo FRONT_RIGHT = new SwerveModuleInfo(1, 2, 11, 315 - 5 - 180/*137.2*/, 1, -1, ModuleLocation.FRONT_RIGHT);
    public static final SwerveModuleInfo FRONT_LEFT = new SwerveModuleInfo(3, 4, 12, 35.5 - 10 - 180/*233.9*/, 1, 1, ModuleLocation.FRONT_LEFT);
    public static final SwerveModuleInfo BACK_RIGHT = new SwerveModuleInfo(5, 6, 13, 316.76 - 10 - 180/*136.1*/, -1, -1, ModuleLocation.BACK_RIGHT);
    public static final SwerveModuleInfo BACK_LEFT = new SwerveModuleInfo(7, 8, 14, 245 - 10 - 180/*53.36*/, -1, 1, ModuleLocation.BACK_LEFT);

    // Shooter
    public static final int SHOOTER_SPINNER_ID_1 = 21;
    public static final int SHOOTER_SPINNER_ID_2 = 22;
    public static final int SHOOTER_PIVOT_ID_1 = 23;
    public static final int SHOOTER_PIVOT_ID_2 = 24;
    public static final int FEEDER_WHEELS_ID = 25;
    public static final int SHOOTER_ENCODER_DIGITAL_PORT_A = 0;
    public static final int SHOOTER_ENCODER_DIGITAL_PORT_B = 1;

    // Intake
    public static final int INTAKE_PIVOT_ID = 41;
    public static final int INTAKE_SPINNER_ID = 42;
    public static final int INTAKE_ENCODER_DIGITAL_PORT_A = 2;
    public static final int INTAKE_ENCODER_DIGITAL_PORT_B = 3;

    // Field Dimensions
    public static final double FIELD_X = 54d + 1d/12d;
    public static final double FIELD_Y = 26d + 7d/12d;
    public static final double SPEAKER_TARGET_HEIGHT = 7.4; //7.83333

    // Various Stats
    public static final double SHOOTER_VELOCITY = 7; //NOTE SHOOT FEET PER SECOND //CHANGE
    public static final double SHOOTER_SET_RPM = 510;
    public static final double SHOOTER_TILT_ENCODER_MAX_VALUE = 1;
    public static final double SHOOTER_TILT_ENCODER_MIN_VALUE = 0;
    public static final double SHOOTER_TILT_ENCODER_OFFSET = -175;
    //public static final double SHOOTER_STANDARD_HEIGHT = 0.458333;
    
    public static final double INTAKE_TILT_ENCODER_MAX_VALUE = 1;
    public static final double INTAKE_TILT_ENCODER_MIN_VALUE = 0;
    public static final double INTAKE_TILT_ENCODER_OFFSET = 85;

    public static final int FAR_BEAM_BREAK_CHANNEL = 9;
    public static final int CLOSE_BEAM_BREAK_CHANNEL = 8;

}
