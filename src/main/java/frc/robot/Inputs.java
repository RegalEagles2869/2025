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

    public static Trigger getSetElevatorSpeedUp() {
        return driver1.a();
    }
    public static Trigger getSetElevatorSpeedDown() {
        return driver1.b();
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
    public static Trigger getBallIntake() {
        return operatorBoard.button(8);
    }
    public static Trigger getBallOuttake() {
        return operatorBoard.button(9);
    }
    public static Trigger getAdjustPivotUp() {
        return operatorBoard.button(10);
    }
    public static Trigger getAdjustPivotDown() {
        return operatorBoard.button(11);
    }
    public static Trigger getRUMBLE() {
        return operatorBoard.button(12);
    }

    public static void RUMBLERUMBLE(double rumble) {
        try {
            controllerLol.setRumble(RumbleType.kBothRumble, rumble);
        } catch (Exception e) {}
    }
}
