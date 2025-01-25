// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BasePosition;
import frc.robot.commands.ChangeElevatorPosition;
import frc.robot.commands.L1Coral;
import frc.robot.commands.L2Coral;
import frc.robot.commands.L3Coral;
import frc.robot.commands.L4Coral;
import frc.robot.commands.SetPivotPosition;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import choreo.trajectory.Trajectory;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private enum Autos {
    Nothing, Something
  }
  
  private double MaxSpeed = 5.5;
  private double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private SendableChooser<Autos> newautopick;
  private Command autoCommand;

  ArrayList<Trajectory> autoTraj = new ArrayList<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("L1Coral", new L1Coral());
    NamedCommands.registerCommand("L2Coral", new L2Coral());
    NamedCommands.registerCommand("L3Coral", new L3Coral());
    NamedCommands.registerCommand("L4Coral", new L4Coral());
    newautopick = new SendableChooser<>();
    newautopick.addOption("Nothing", Autos.Nothing);
    newautopick.addOption("SillySix", Autos.Something);

    Shuffleboard.getTab("auto").add("auto", newautopick).withPosition(0, 0).withSize(3, 1);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Inputs.getAdjustElevatorDown().whileTrue(new ChangeElevatorPosition(-.1));
    Inputs.getAdjustElevatorUp().whileTrue(new ChangeElevatorPosition(.1));
    Inputs.getAdjustPivotUp().whileTrue(new ChangeElevatorPosition(-.1));
    Inputs.getAdjustPivotDown().whileTrue(new ChangeElevatorPosition(.1));
    Inputs.getL1Coral().onTrue(new L1Coral());
    Inputs.getL2Coral().onTrue(new L2Coral());
    Inputs.getL3Coral().onTrue(new L3Coral());
    Inputs.getL4Coral().onTrue(new L4Coral());
    Inputs.getFloorPosition().onTrue(new BasePosition());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoCommand == null) {
      autoCommand = generateAutoCommand();
    }
    return autoCommand;
  }

  
  public void generateTrajectories(String name){
    List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(name);
    for(PathPlannerPath path:paths){
      autoTraj.add(TrajectoryGenerator.generateTrajectory(path.getPathPoses(), new TrajectoryConfig(MaxSpeed, MaxAngularRate)));
      // autoTraj.add(path.getPathPoses());   
    }
  }
  
  private Command generateAutoCommand(){
    switch(newautopick.getSelected()){
      case Nothing:
        return new SequentialCommandGroup(new WaitCommand(50000));
      default:
        return new WaitCommand(100);
    }
  }
}
