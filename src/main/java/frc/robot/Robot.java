// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ElevatorResetPosition;
import frc.robot.commands.RumbleRumble;

public class Robot extends TimedRobot {
  public enum RobotState {
    DISABLED, TELEOP, AUTON, TEST
  }
  private Command m_autonomousCommand;
  private static RobotState state = RobotState.DISABLED;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    Inputs.getRUMBLE().whileTrue(new RumbleRumble().ignoringDisable(true));
    Inputs.getElevatorResetPosition().whileTrue(new ElevatorResetPosition(0).ignoringDisable(true));
    state = RobotState.DISABLED;
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    state = RobotState.AUTON;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    state = RobotState.TELEOP;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    state = RobotState.TEST;
    // if (getPos() > Constants.ElevatorConstants.floorPosition) {
    //   CommandScheduler.getInstance().schedule(null);
    // }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    state = RobotState.TEST;
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  public static RobotState getState() {
    return state;
  }
}
