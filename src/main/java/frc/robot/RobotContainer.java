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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CenterAtAprilTag;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.ElevatorToFloor;
import frc.robot.commands.ElevatorToFloorFinal;
import frc.robot.commands.L1Coral;
import frc.robot.commands.L2Coral;
import frc.robot.commands.L3Coral;
import frc.robot.commands.L4Coral;
import frc.robot.commands.L4ElevatorPosition;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.RumbleRumble;
import frc.robot.commands.SetClimberPosition;
import frc.robot.commands.SetClimberSpeed;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetElevatorPositionInstant;
import frc.robot.commands.ElevatorAndSwerve;
import frc.robot.commands.ElevatorResetPosition;
import frc.robot.commands.SetElevatorSpeed;
import frc.robot.commands.SetIntakeSpeed;
import frc.robot.commands.SetIntakeSpeedWait;
import frc.robot.commands.SourceIntake;
import frc.robot.commands.MoveSwerve;
import frc.robot.commands.TestCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
	private ElevatorSubsystem elevator;
	// speed
	private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
	// second
	// max angular velocity

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(
					MaxSpeed * Constants.OperatorConstants.driveDeadBand
							* Constants.OperatorConstants.speedMultiplier)
			.withRotationalDeadband(MaxAngularRate * Constants.OperatorConstants.rotationDeadBand
					* Inputs.getMultiplier() * Constants.OperatorConstants.speedMultiplier)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
	// motors
	private final SwerveRequest.FieldCentric driveSlow = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * Constants.OperatorConstants.driveDeadBand
					* Constants.OperatorConstants.speedMultiplierSlowMode)
			.withRotationalDeadband(MaxAngularRate * Constants.OperatorConstants.rotationDeadBand
					* Inputs.getMultiplier() * Constants.OperatorConstants.speedMultiplierSlowMode)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
	// motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final Telemetry logger = new Telemetry(MaxSpeed);

	private final CommandXboxController joystick = new CommandXboxController(0);

	public CommandSwerveDrivetrain drivetrain;

	public RobotContainer() {
		drivetrain = TunerConstants.createDrivetrain();
		elevator = ElevatorSubsystem.getInstance();
		elevator.setEncoderPosition(0);
		Field2d field = new Field2d();
		// Do this in either robot or subsystem init
		SmartDashboard.putData("Field", field);
		NamedCommands.registerCommand("TestCommand", new TestCommand("fishing hahaha"));
		NamedCommands.registerCommand("L1", new L1Coral());
		NamedCommands.registerCommand("L2", new L2Coral());
		NamedCommands.registerCommand("L2Pos", new SetElevatorPositionInstant(Constants.ElevatorConstants.l2Position));
		NamedCommands.registerCommand("L3", new L3Coral());
		NamedCommands.registerCommand("L4", new L4Coral());
		NamedCommands.registerCommand("Limelight", new SequentialCommandGroup(new MoveSwerve(0, 0, 0), new CenterAtAprilTag(true), new ParallelDeadlineGroup(new WaitCommand(Constants.SwerveConstants.waitTheyDontLoveYouLikeILoveYou), new MoveSwerve(0, Constants.SwerveConstants.forwardForAuto, 0))));
		NamedCommands.registerCommand("StopSwerve", new MoveSwerve(0, 0, 0));
		// NamedCommands.registerCommand("Order1", new L1Coral());
		// NamedCommands.registerCommand("Order2", new L2Coral());
		// NamedCommands.registerCommand("Order2", new L2Coral());
		// NamedCommands.registerCommand("Order3", new L3Coral());
		// NamedCommands.registerCommand("Order4", new L4Coral());
		// NamedCommands.registerCommand("SourceIntake", new SourceIntake());
		NamedCommands.registerCommand("StopAndWait", new SequentialCommandGroup(new MoveSwerve(0, 0, 0), new WaitCommand(1)));
		// Do this in either robot periodic or subsystem periodic
		field.setRobotPose(drivetrain.getNegativePose());
		configureBindings();
	}

	private void configureBindings() {
		Inputs.getResetGyro().onTrue(new ResetGyro());

		Inputs.getElevatorDown().onTrue(new ElevatorToFloor());
		// Inputs.getElevatorL2().onTrue(new L2Coral());
		// Inputs.getElevatorL3().onTrue(new L3Coral());
		// Inputs.getElevatorL4().onTrue(new L4Coral());
		Inputs.getElevatorL2().onTrue(new SetElevatorPosition(Constants.ElevatorConstants.l2Position));
		Inputs.getElevatorL3().onTrue(new SetElevatorPosition(Constants.ElevatorConstants.l3Position));
		Inputs.getElevatorL4().onTrue(new SetElevatorPosition(Constants.ElevatorConstants.l4Position));
		// Inputs.getClimberNeutral().onTrue(new SetClimberPosition(Constants.ClimberConstants.floorPosition));
		// Inputs.getClimberGood().onTrue(new SetClimberPosition(Constants.ClimberConstants.goodPosition));

		Inputs.getElevatorSpeedUp().whileTrue(new SetElevatorSpeed(.2));
		Inputs.getElevatorSpeedDown().whileTrue(new SetElevatorSpeed(-.2));
		Inputs.getIntakeIn().whileTrue(new SetIntakeSpeed(-Constants.CoralConstants.intakeSpeed));
		Inputs.getIntakeOut().whileTrue(new SetIntakeSpeed(Constants.CoralConstants.intakeSpeed));

		Inputs.getClimberUp().whileTrue(new SetClimberSpeed(1));
		Inputs.getClimberDown().whileTrue(new SetClimberSpeed(-1));
		Inputs.getRUMBLE().whileTrue(new RumbleRumble());
		// Inputs.getTest().onTrue(new DriveToPose(Constants.FieldConstants.coralPose1));
		// Inputs.getTest().onTrue(new L2Coral());
		Inputs.getTest().onTrue(new CenterAtAprilTag(true));
		// Inputs.getTest().onTrue(new SetIntakeSpeedWait(Constants.CoralConstants.intakeSpeed));
		Inputs.getElevatorResetPosition().onTrue(new ElevatorResetPosition(0));

		// Note that X is defi,m ned as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.

		// drivetrain.setDefaultCommand(
		// // Drivetrain will execute this command periodically
		// drivetrain.applyRequest(() ->
		// drive.withVelocityX(Inputs.getTranslationX() * MaxSpeed *
		// Constants.OperatorConstants.speedMultiplier) // Drive forward with negative Y
		// // (forward)
		// .withVelocityY(Inputs.getTranslationY() * MaxSpeed *
		// Constants.OperatorConstants.speedMultiplier) // Drive left with negative X
		// (left)
		// .withRotationalRate(Inputs.getRotation() * MaxAngularRate *
		// Constants.OperatorConstants.speedMultiplier) // Drive counterclockwise with
		// // negative X (left)
		// ));
		// elevator.getPos() > 
		drivetrain.setDefaultCommand(
			// Drivetrain will execute this command periodically
			drivetrain.applyRequest(() -> drive
					.withVelocityX(
							Inputs.getTranslationX() * MaxSpeed * Constants.OperatorConstants.speedMultiplier)
					// Drive forward with negative Y (forward)
					.withVelocityY(
							Inputs.getTranslationY() * MaxSpeed * Constants.OperatorConstants.speedMultiplier)
					// Drive left with negative X (left)
					.withRotationalRate(
							Inputs.getRotation() * MaxAngularRate * Constants.OperatorConstants.speedMultiplier)
			// Drive counterclockwise with negative X (left)
			)
		);

		
		new ElevatorAndSwerve(() -> true).whileTrue(
			// joystick.x().whileTrue(
			drivetrain.applyRequest(() -> driveSlow
					.withVelocityX(Inputs.getTranslationX() * MaxSpeed
							* Constants.OperatorConstants.speedMultiplierSlowMode) // Drive
					// forward
					// with
					// negative
					// Y
					// (forward)
					.withVelocityY(Inputs.getTranslationY() * MaxSpeed
							* Constants.OperatorConstants.speedMultiplierSlowMode) // Drive
					.withRotationalRate(Inputs.getRotation() * MaxAngularRate
							* Constants.OperatorConstants.speedMultiplierSlowMode) // Drive
			// counterclockwise
			// with
			// negative
			// X
			// (left)
			)
		);
		joystick.leftTrigger().whileTrue(
			// joystick.x().whileTrue(
			drivetrain.applyRequest(() -> driveSlow
					.withVelocityX(Inputs.getTranslationX() * MaxSpeed
							* Constants.OperatorConstants.speedMultiplierSlowMode) // Drive
					// forward
					// with
					// negative
					// Y
					// (forward)
					.withVelocityY(Inputs.getTranslationY() * MaxSpeed
							* Constants.OperatorConstants.speedMultiplierSlowMode) // Drive
					.withRotationalRate(Inputs.getRotation() * MaxAngularRate
							* Constants.OperatorConstants.speedMultiplierSlowMode) // Drive
			// counterclockwise
			// with
			// negative
			// X
			// (left)
			));
		joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
		joystick.b().whileTrue(drivetrain.applyRequest(
				() -> point.withModuleDirection(
						new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

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
		// return drivetrain.moveTo(new Pose2d(1, 0, new Rotation2d(0)));
		// return new ParallelDeadlineGroup(new WaitCommand(5), new DriveToPose(new Pose2d(0, .5, new Rotation2d(0))));
		// drivetrain.getAuto("TheSilly6");
		// return new WaitCommand(0);
		try {
			//  PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("TestPath");
			 return drivetrain.getAuto("TheSilly6");
			// return new L2Coral();
		}
		catch (Exception e) {
			return new WaitCommand(0);
		}
	}
}