// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorFloor extends Command {
  private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
  private double position;

  /** Creates a new SetPivotPosition. */
  public SetElevatorFloor() {
    // addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setTarPos(Constants.ElevatorConstants.floorPosition);
    // elevator.setToBrake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("a");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setToBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return elevator.isAtPosition();
    return elevator.getPos() < Constants.ElevatorConstants.slowPose;
    // return true;
  }
}
