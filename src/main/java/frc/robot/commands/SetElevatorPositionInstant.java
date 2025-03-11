// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPositionInstant extends InstantCommand {
  private ElevatorSubsystem elevator = ElevatorSubsystem.justGetInstance();
  private double position;
  /** Creates a new SetPivotPosition. */
  public SetElevatorPositionInstant(double position) {
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  
  @Override
  public void execute() {
    elevator.setTarPos(position);
  }
}
