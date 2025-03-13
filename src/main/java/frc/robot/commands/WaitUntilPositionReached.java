// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WaitUntilPositionReached extends Command {
  /** Creates a new WaitUntilPositionReached. 
   *  This command assumes the robot is currently moving forward.
   * When it is created the position is recorded and the command ends when the position has moved
   * forwrd a given number of inches FROM THE ROBOTS PERSEPCTIVE.
   * Note that this command does not move the robot. It is only used for sensing robot motion to allow for a precise
   * distance forward.
   * 
  */

  private CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
  private double goalMeters;
  private Pose2d start;

  /**
   * 
   * @param m number of meters.
   */
  public WaitUntilPositionReached(double m) {
    this.goalMeters = m;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = swerve.getPose();
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
    double dx =swerve.getState().Pose.getX() - start.getX();
    double dy =swerve.getState().Pose.getY() - start.getY();

    return 
      dx * dx + 
      dy * dy > 
      goalMeters * goalMeters;
  }
}
