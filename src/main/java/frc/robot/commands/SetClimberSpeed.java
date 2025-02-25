// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetClimberSpeed extends Command {
  private ClimberSubsystem climber = ClimberSubsystem.getInstance();
  private double speed;
  /** Creates a new SetElevatorSpeed. */
  public SetClimberSpeed(double speed) {
    addRequirements(climber);
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((climber.getPosition() < Constants.ElevatorConstants.maxPosition && speed > 0)
      || (climber.getPosition() > Constants.ElevatorConstants.floorPosition && speed < 0)) {
      // climber.setSpeed(0);
      // return true;
    }

    return false;
  }
}
