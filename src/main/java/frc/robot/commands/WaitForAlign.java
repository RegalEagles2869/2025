// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WaitForAlign extends Command {
  private boolean isRight;
  /** Creates a new WaitForAlign. */
  public WaitForAlign(boolean isRight) {
    this.isRight = isRight;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setBool(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (isRight)
    //   return LimelightHelpers.getRx() == 0 && LimelightHelpers.getRz() == 0 && LimelightHelpers.getThetaLeft() == 0 && LimelightHelpers.getTV("limelight-noor");
    // else
    //   return LimelightHelpers.getLx() == 0 && LimelightHelpers.getLz() == 0 && LimelightHelpers.getThetaLeft() == 0 && LimelightHelpers.getTV("limelight-noor");
    if (isRight) return LimelightHelpers.getFinishedRight();
    else return LimelightHelpers.getFinishedLeft();
  }
}
