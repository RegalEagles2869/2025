// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.TestCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.OperatorConstants.driveDeadBand * Constants.OperatorConstants.speedMultiplier).withRotationalDeadband(MaxAngularRate * Constants.OperatorConstants.rotationDeadBand * Inputs.getMultiplier() * Constants.OperatorConstants.speedMultiplier)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentric driveSlow = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.OperatorConstants.driveDeadBand * Constants.OperatorConstants.speedMultiplierSlowMode).withRotationalDeadband(MaxAngularRate * Constants.OperatorConstants.rotationDeadBand * Inputs.getMultiplier() * Constants.OperatorConstants.speedMultiplierSlowMode)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        Field2d field = new Field2d();
        // Do this in either robot or subsystem init
        SmartDashboard.putData("Field", field);
        NamedCommands.registerCommand("TestCommand", new TestCommand());
        // Do this in either robot periodic or subsystem periodic
        field.setRobotPose(drivetrain.getNegativePose());
        configureBindings();
    }

    private void configureBindings() {
        Inputs.getResetGyro().onTrue(new ResetGyro());
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(Inputs.getTranslationX() * MaxSpeed * Constants.OperatorConstants.speedMultiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(Inputs.getTranslationY() * MaxSpeed * Constants.OperatorConstants.speedMultiplier) // Drive left with negative X (left)
                    .withRotationalRate(Inputs.getRotation() * MaxAngularRate * Constants.OperatorConstants.speedMultiplier) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.x().whileTrue(
            drivetrain.applyRequest(() ->
                driveSlow.withVelocityX(Inputs.getTranslationX() * MaxSpeed * Constants.OperatorConstants.speedMultiplierSlowMode) // Drive forward with negative Y (forward)
                    .withVelocityY(Inputs.getTranslationY() * MaxSpeed * Constants.OperatorConstants.speedMultiplierSlowMode) // Drive left with negative X (left)
                    .withRotationalRate(Inputs.getRotation() * MaxAngularRate * Constants.OperatorConstants.speedMultiplierSlowMode) // Drive counterclockwise with negative X (left)
            )
        );
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // return new PathPlannerAuto("fishAuto");
        try{
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

            // Create a path following command using AutoBuilder. This will also trigger event markers.
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}
