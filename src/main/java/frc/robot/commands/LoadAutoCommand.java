// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LoadAutoCommand extends InstantCommand {
  private RobotContainer r;

  public LoadAutoCommand() {
    r = Robot.m_robotContainer;
    runsWhenDisabled();
    // Use addRequirements() here to declare subsystem dependencies.
  }

boolean hasRun = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!hasRun){
      hasRun = true;
    }
    r.generateAndLoad();
  }
}
