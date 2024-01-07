package frc.robot.subsystems.swerve;

import frc.robot.subsystems.swerve.SwerveModules.ModuleLocation;
import frc.robot.utils.VectorR;

public class SwerveModuleInfo {
    public final int DRIVE_ID;
    public final int TURN_ID;
    public final double ABS_ENCODER_MAX_VALUE;
    public final double ABS_ENCODER_VALUE_WHEN_STRAIGHT;
    public final int ENCODER_ID;
    public final double X;
    public final double Y;
    public final SwerveModules.ModuleLocation LOCATION;
    public final double MODULE_TANGENT_DEG;

    public SwerveModuleInfo(int drive_motor_CAN_ID, int angle_motor_CAN_ID, int encoder_CAN_ID, double abs_encoder_max_value,
            double abs_encoder_value_when_wheel_straight, double x, double y, ModuleLocation location) {
        DRIVE_ID = drive_motor_CAN_ID;
        TURN_ID = angle_motor_CAN_ID;
        ABS_ENCODER_MAX_VALUE = abs_encoder_max_value;
        ABS_ENCODER_VALUE_WHEN_STRAIGHT = abs_encoder_value_when_wheel_straight;
        ENCODER_ID = encoder_CAN_ID;
        X = x;
        Y = y;
        LOCATION = location;
        MODULE_TANGENT_DEG = VectorR.fromCartesian(x, y).getAngle() + 90;
    }
}