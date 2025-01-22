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

    public static Trigger getAdjustElevatorUp() {
        return operatorBoard.button(1);
    }
    public static Trigger getAdjustElevatorDown() {
        return operatorBoard.button(2);
    }
    public static Trigger getElevatorFloorPosition() {
        return operatorBoard.button(3);
    }
}
