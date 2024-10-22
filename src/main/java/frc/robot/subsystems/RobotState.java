// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;

/** Add your docs here. */
public class RobotState {
    private static RobotConfiguration chosenConfiguration = RobotConfiguration.SHOOT_SPEAKER;
    private static RobotConfiguration robotConfiguration = RobotConfiguration.TRAVEL;

    public static void setRobotState(RobotConfiguration robotConfig){
        robotConfiguration = robotConfig;
    } 

    public static RobotConfiguration getRobotConfiguration(){
        return robotConfiguration;
    }

    public static void setChosenConfiguration(RobotConfiguration robotConfig){
        chosenConfiguration = robotConfig;
    }

    public static RobotConfiguration getChosenRobotConfiguration(){
        return chosenConfiguration;
    }

    public enum RobotConfiguration {
        SHOOT_SPEAKER(IntakePosition.RETRACTED, ShooterSpeed.SPEAKER, ShooterAngle.NONE),
        SHOOT_OVER(IntakePosition.RETRACTED, ShooterSpeed.SPEAKER, ShooterAngle.TOP),
        PASS(IntakePosition.RETRACTED, ShooterSpeed.SPEAKER, ShooterAngle.PASS),
        SHOOT_AMP(IntakePosition.RETRACTED, ShooterSpeed.TRAVEL, ShooterAngle.AMP),
        INTAKE(IntakePosition.EXTENDED, ShooterSpeed.TRAVEL, ShooterAngle.TRAVEL),
        TRAVEL(IntakePosition.RETRACTED, ShooterSpeed.TRAVEL, ShooterAngle.TRAVEL),
        TRAP(IntakePosition.RETRACTED, ShooterSpeed.TRAVEL, ShooterAngle.TRAP),
        CLIMB_UP(IntakePosition.RETRACTED, ShooterSpeed.TRAVEL, ShooterAngle.CLIMB_UP),
        CLIMB_DOWN(IntakePosition.RETRACTED, ShooterSpeed.TRAVEL, ShooterAngle.CLIMB_DOWN);
        
        public final ShooterSpeed shooterSpeed;
        public final ShooterAngle shooterAngle;
        public final IntakePosition intakePos;
        
        private RobotConfiguration( 
        IntakePosition intakePos,
        ShooterSpeed shooterSpeed,
        ShooterAngle shooterAngle) {
        this.shooterSpeed = shooterSpeed;
        this.intakePos = intakePos;
        this.shooterAngle = shooterAngle;
        }
    }
}
