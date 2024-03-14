// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
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

    public static ShooterSpeed getShooterSpeed(){
        return chosenConfiguration.shooterSpeed;
    }

    public enum RobotConfiguration {
        SHOOT_SPEAKER(ElevatorPosition.TRAVEL, IntakePosition.RETRACTED, ShooterSpeed.SPEAKER, ShooterAngle.NONE),
        SHOOT_AMP(ElevatorPosition.TRAVEL, IntakePosition.AMP, ShooterSpeed.TRAVEL, ShooterAngle.NONE),
        SHOOT_TRAP(ElevatorPosition.TRAVEL, IntakePosition.RETRACTED, ShooterSpeed.TRAP, ShooterAngle.NONE),
        INTAKE(ElevatorPosition.TRAVEL, IntakePosition.EXTENDED, ShooterSpeed.TRAVEL, ShooterAngle.NONE),
        TRAVEL(ElevatorPosition.TRAVEL, IntakePosition.RETRACTED, ShooterSpeed.TRAVEL, ShooterAngle.NONE);
        
        public final ShooterSpeed shooterSpeed;
        public final ShooterAngle shooterAngle;
        public final ElevatorPosition elevatorPos;
        public final IntakePosition intakePos;
        
        private RobotConfiguration( 
        ElevatorPosition elevatorPos,
        IntakePosition intakePos,
        ShooterSpeed shooterSpeed,
        ShooterAngle shooterAngle) {
        this.shooterSpeed = shooterSpeed;
        this.elevatorPos = elevatorPos;
        this.intakePos = intakePos;
        this.shooterAngle = shooterAngle;
        }
    }
}
