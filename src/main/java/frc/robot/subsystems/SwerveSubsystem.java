// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class SwerveSubsystem extends SubsystemBase {

  private static SwerveSubsystem instance;

  public static SwerveSubsystem getInstance() {
    if (instance == null) instance = new SwerveSubsystem();
    return instance;
  }

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {}
  
  /**
   * Gets the autonomous command from Path Planner
   * @param autoName name of the autonomous
   * @return PathPlannerAuto to run
   */
  public Command getAuto(String autoName){
    resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(autoName));
    return new PathPlannerAuto(autoName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
