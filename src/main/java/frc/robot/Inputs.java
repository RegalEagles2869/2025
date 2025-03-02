// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
    
    // DID YOU JUST UPDATE CTRE SWERVE? Remember to invert the drive motors in TunerConstants
    public static double getTranslationX() {
        return driver1.getLeftY();
    }
    public static double getTranslationY() {
        return driver1.getLeftX();
    }
    public static double getRotation() {
        double speed =  -driver1.getRightX();
        return speed;
    }

    public static double getMultiplier() {
        return Constants.OperatorConstants.speedMultiplier;
    }

    public static Trigger getElevatorL4() {
        return operatorBoard.button(1);
    }
    public static Trigger getElevatorL2() {
        return operatorBoard.button(2);
    }
    public static Trigger getElevatorL3() {
        return operatorBoard.button(3);
    }
    public static Trigger getClimberNeutral() {
        return operatorBoard.button(4);
    }
    public static Trigger getClimberGood() {
        return operatorBoard.button(5);
    }
    public static Trigger getElevatorSpeedUp() {
        return operatorBoard.button(6);
    }
    public static Trigger getElevatorSpeedDown() {
        return operatorBoard.button(7);
    }
    public static Trigger getIntakeIn() {
        return operatorBoard.button(8);
    }
    public static Trigger getIntakeOut() {
        return operatorBoard.button(9);
    }
    public static Trigger getClimberUp() {
        return operatorBoard.button(10);
    }
    public static Trigger getClimberDown() {
        return operatorBoard.button(11);
    }
    public static Trigger getRUMBLE() {
        return operatorBoard.button(12);
    }
    public static Trigger getTest() {
        return operatorBoard.button(13);
    }
    public static Trigger getElevatorDown() {
        return operatorBoard.button(14);
    }

    public static void RUMBLERUMBLE(double rumble) {
        try {
            controllerLol.setRumble(RumbleType.kBothRumble, rumble);
        } catch (Exception e) {}
    }
}
