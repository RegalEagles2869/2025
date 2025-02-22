// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetPivotSpeed extends Command {
  private PivotSubsystem pivot = PivotSubsystem.getInstance();
  private double speed;
  /** Creates a new SetPivotSpeed. */
  public SetPivotSpeed(double speed) {
    addRequirements(pivot);
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((pivot.getPosition() < Constants.PivotConstants.maxPosition && speed > 0)
      || (pivot.getPosition() > Constants.PivotConstants.floorPosition && speed < 0)) {
        // pivot.setSpeed(0);
        // return true;
      }
    return false;
  }
}
