// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class ElevatorAndSwerve extends Trigger {
    private static ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    
    public ElevatorAndSwerve(BooleanSupplier condition) {
        super(() -> elevator.getPos() > Constants.ElevatorConstants.floorPosition);
    }
    // private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    // public ElevatorAndSwerve() {
    //     super(() -> elevator.getPos() > Constants.ElevatorConstants.l3Position);
    // }
}
