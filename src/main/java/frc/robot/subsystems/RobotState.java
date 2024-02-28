// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.ShooterSubsystem.ShooterPosition;

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
        SHOOT_SPEAKER(ElevatorPosition.TRAVEL, IntakePosition.RETRACTED, ShooterPosition.MANUAL),
        SHOOT_AMP(ElevatorPosition.AMP, IntakePosition.OUT_OF_THE_WAY, ShooterPosition.AMP),
        SHOOT_TRAP(ElevatorPosition.TRAVEL, IntakePosition.RETRACTED, ShooterPosition.TRAP),
        INTAKE(ElevatorPosition.TRAVEL, IntakePosition.EXTENDED, ShooterPosition.TRAVEL),
        TRAVEL(ElevatorPosition.TRAVEL, IntakePosition.RETRACTED, ShooterPosition.TRAVEL);

        public final ShooterPosition shooterPos;
        public final ElevatorPosition elevatorPos;
        public final IntakePosition intakePos;
        
        private RobotConfiguration( 
        ElevatorPosition elevatorPos,
        IntakePosition intakePos,
        ShooterPosition shooterPos) {
        this.shooterPos = shooterPos;
        this.elevatorPos = elevatorPos;
        this.intakePos = intakePos;
        }
    }
}
