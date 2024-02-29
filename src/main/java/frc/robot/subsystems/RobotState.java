// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.ShooterSubsystem.ShooterPosition;
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
        SHOOT_SPEAKER(ElevatorPosition.TRAVEL, IntakePosition.RETRACTED, ShooterPosition.MANUAL, ShooterSpeed.SPEAKER, ShooterAngle.NONE),
        SHOOT_AMP(ElevatorPosition.TRAVEL, IntakePosition.AMP, ShooterPosition.TRAVEL, ShooterSpeed.TRAVEL, ShooterAngle.NONE),
        SHOOT_TRAP(ElevatorPosition.TRAVEL, IntakePosition.RETRACTED, ShooterPosition.TRAP, ShooterSpeed.TRAP, ShooterAngle.NONE),
        INTAKE(ElevatorPosition.TRAVEL, IntakePosition.EXTENDED, ShooterPosition.TRAVEL, ShooterSpeed.TRAVEL, ShooterAngle.NONE),
        TRAVEL(ElevatorPosition.TRAVEL, IntakePosition.RETRACTED, ShooterPosition.TRAVEL, ShooterSpeed.TRAVEL, ShooterAngle.NONE),
        SUBWOOFER(ElevatorPosition.TRAVEL, IntakePosition.RETRACTED, ShooterPosition.MANUAL, ShooterSpeed.SPEAKER, ShooterAngle.SUBWOOFER),
        POST(ElevatorPosition.TRAVEL, IntakePosition.RETRACTED, ShooterPosition.MANUAL, ShooterSpeed.SPEAKER, ShooterAngle.POST),
        FAR_POST(ElevatorPosition.TRAVEL, IntakePosition.RETRACTED, ShooterPosition.MANUAL, ShooterSpeed.SPEAKER, ShooterAngle.FAR_POST);

        public final ShooterSpeed shooterSpeed;
        public final ShooterAngle shooterAngle;
        public final ShooterPosition shooterPos;
        public final ElevatorPosition elevatorPos;
        public final IntakePosition intakePos;
        
        private RobotConfiguration( 
        ElevatorPosition elevatorPos,
        IntakePosition intakePos,
        ShooterPosition shooterPos,
        ShooterSpeed shooterSpeed,
        ShooterAngle shooterAngle) {
        this.shooterSpeed = shooterSpeed;
        this.shooterPos = shooterPos;
        this.elevatorPos = elevatorPos;
        this.intakePos = intakePos;
        this.shooterAngle = shooterAngle;
        }
    }
}
