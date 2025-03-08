// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterAtAprilTag extends Command {
  private CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
  private boolean finished = false;
  private double degrees  = 0;
  private double tarX;
  private double tarY;
  /** Creates a new CenterAtAprilTag. */
  public CenterAtAprilTag(boolean isLeft) {
    addRequirements(swerve);
    if (isLeft) {
      tarX = Constants.SwerveConstants.xPosLeft;
      tarY = Constants.SwerveConstants.yPosLeft;
    }
    else {
      tarX = Constants.SwerveConstants.xPosRight;
      tarY = Constants.SwerveConstants.yPosRight;
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    int id = (int)LimelightHelpers.getFiducialID("limelight-noor");
    for (int i = 0; i < Constants.SwerveConstants.ids.length; i++) {
      if (id == Constants.SwerveConstants.ids[i]) {
        degrees = Constants.SwerveConstants.ids[i];
      }
    }
  }
  //IMU IMU IMU IMU IMU
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = 0;
    double y = 0;
    double theta = 0;
    double realX = LimelightHelpers.getTX("limelight-noor");
    double RealY = LimelightHelpers.getTY("limelight-noor");
    if (Math.abs(swerve.getState().Pose.getRotation().getDegrees() - degrees) > Constants.SwerveConstants.rotationErrorLimelight) {
      if (swerve.getState().Pose.getRotation().getDegrees() > degrees) theta -= Constants.SwerveConstants.rotIncLimelight;
      else theta += Constants.SwerveConstants.rotIncLimelight;
    }
    else {
      if (Math.abs(realX - tarX) > Constants.SwerveConstants.xErrorLimelight) {
        if (x > tarX) x -= Constants.SwerveConstants.xIncLimelight;
        else x += Constants.SwerveConstants.xIncLimelight;
      }
      if (Math.abs(RealY - tarY) > Constants.SwerveConstants.yErrorLimelight) {
        if (y > tarY) y -= Constants.SwerveConstants.yIncLimelight;
        else y += Constants.SwerveConstants.yIncLimelight;
      }
    }
    swerve.setControl(
      m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(y, x, theta))
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
