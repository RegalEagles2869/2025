// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your inputs here. */
public class Inputs {
    private static final XboxController controllerLol = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
    private static final CommandXboxController driver1 = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort); 
    private static final CommandGenericHID operatorBoard = new CommandGenericHID(Constants.OperatorConstants.kOperatorControllerPort); 

    public static Trigger getResetGyro() {
        return driver1.y();
    }
    
    public static boolean getSlowMode(){
        return driver1.getHID().getRightBumper()||driver1.getHID().getLeftBumper();
    }
    // DID YOU JUST UPDATE CTRE SWERVE? Remember to invert the drive motors in TunerConstants
    public static double getTranslationX() {
        return driver1.getLeftY();
    }
    public static double getTranslationY() {
        return driver1.getLeftX();
    }
    public static double getRotation() {
        double speed =  -driver1.getRightX();
        if(getSlowMode()){
            return speed * Constants.OperatorConstants.speedMultiplierSlowMode;
        }
        return speed * Constants.OperatorConstants.speedMultiplier;
    }

    public static double getMultiplier() {
        if (getSlowMode()) return Constants.OperatorConstants.speedMultiplierSlowMode;
        return Constants.OperatorConstants.speedMultiplier;
    }


    public static Trigger getAdjustElevatorUp() {
        return operatorBoard.button(1);
    }
    public static Trigger getAdjustElevatorDown() {
        return operatorBoard.button(2);
    }
    public static Trigger getFloorPosition() {
        return operatorBoard.button(3);
    }
    public static Trigger getL1Coral() {
        return operatorBoard.button(4);
    }
    public static Trigger getL2Coral() {
        return operatorBoard.button(5);
    }
    public static Trigger getL3Coral() {
        return operatorBoard.button(6);
    }
    public static Trigger getL4Coral() {
        return operatorBoard.button(7);
    }
}
