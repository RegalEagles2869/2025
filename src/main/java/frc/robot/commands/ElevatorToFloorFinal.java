// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToFloorFinal extends Command {
  private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

  /** Creates a new ElevatorToFloor. */
  public ElevatorToFloorFinal() {
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.bottomed = false;
    elevator.low = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.ending();
    // if (elevator.getTarPos() < Constants.ElevatorConstants.slowPose) {
    //   // elevator.setSpeed(-.01);
    // } else {
    //   elevator.setSpeedWithoutPID(-.5);
    //   SmartDashboard.putString("state", "speed down");

    //   // elevator.low = false;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (elevator.getCurrent() >= Constants.ElevatorConstants.maxCurrent
    // || elevator.getPosition() <= Constants.ElevatorConstants.floorPosition) {
    // elevator.setSpeedWithoutPID(0);
    // elevator.setToCoast();
    // return true;
    // }
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    System.out.println("balls to the walls");
    return elevator.bottomed;
  }
}
