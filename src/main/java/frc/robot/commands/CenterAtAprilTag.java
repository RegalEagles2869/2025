// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Inputs;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * @author lime.
 */
public class CenterAtAprilTag extends Command {
  private CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
  // private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
  private boolean finished = false;
  private double tarX;
  private double tarY;
  private double tarTheta;
  
  private double x = 0;
  private double z = 0;
  private double theta = 0;
  
	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
	// speed
	private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
  
	private final SwerveRequest.RobotCentric driveLime = new SwerveRequest.RobotCentric()
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive

  /** Creates a new CenterAtAprilTag. */
  public CenterAtAprilTag(boolean isLeft) {
    if (isLeft) {
      tarX = Constants.SwerveConstants.xPosLeft;
      tarY = Constants.SwerveConstants.zPosLeft;
      tarTheta = Constants.SwerveConstants.leftThetaPos;
    }
    else {
      tarX = Constants.SwerveConstants.xPosRight;
      tarY = Constants.SwerveConstants.zPosRight;
      tarTheta = Constants.SwerveConstants.leftThetaPos;
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    // int id = (int)LimelightHelpers.getFiducialID("limelight-noor");
    // for (int i = 0; i < Constants.SwerveConstants.ids.length; i++) {
    //   if (id == Constants.SwerveConstants.ids[i]) {
    //     degrees = Constants.SwerveConstants.ids[i];
    //   }
    // }
  }
  // IMU IMU IMU IMU IMU
  // x is left right, z is forward backward, and yaw is rotation
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] coords = LimelightHelpers.getTargetPose_CameraSpace("limelight-noor");
    x = 0;
    z = 0;
    theta = 0;

    boolean isXDone = true;
    boolean isZDone = true;
    boolean isThetaDone = true;
    //double realX = LimelightHelpers.getTX("limelight-noor");
    //double realY = LimelightHelpers.getTA("limelight-noor");
    //x y z f y f
    double realX = coords[0];
    double realZ = coords[2];
    double realTheta = coords[4];

    if (Math.abs(realTheta - tarTheta) > Constants.SwerveConstants.rotationErrorLimelight) {
      if (tarTheta > realTheta) theta -= Constants.SwerveConstants.rotIncLimelight;
      else theta += Constants.SwerveConstants.rotIncLimelight;
      isXDone = false;
      isZDone = false;
      isThetaDone = false;
    }
    else {
      if (Math.abs(realX - tarX) > Constants.SwerveConstants.xErrorLimelight) {
        isXDone = false;
        if (x > tarX) x += Constants.SwerveConstants.xIncLimelight;
        else x -= Constants.SwerveConstants.xIncLimelight;
      }
      if (Math.abs(realZ - tarY) > Constants.SwerveConstants.zErrorLimelight) {
        isZDone = false;
        if (z > tarY) z -= Constants.SwerveConstants.zIncLimelight;
        else z += Constants.SwerveConstants.zIncLimelight;
      }
    }
    SmartDashboard.putNumber("thetaPos", realTheta);
    SmartDashboard.putNumber("xPos", realX);
    SmartDashboard.putNumber("zPos", realZ);
    SmartDashboard.putBooleanArray("progress check hehe", new boolean[]{isXDone, isZDone, isThetaDone});
    swerve.applyRequest(() -> driveLime
      .withVelocityX(0) // Drive
      // forward
      // with
      // negative
      // Y
      // (forward)
      .withVelocityY(10) // Drive
      .withRotationalRate(0)
    ); // Drive
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
