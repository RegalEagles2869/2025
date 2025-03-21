// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot.RobotState;
import frc.robot.commands.CenterAtAprilTag;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.ElevatorToFloor;
import frc.robot.commands.ElevatorToFloorFinal;
import frc.robot.commands.L1Coral;
import frc.robot.commands.L2Coral;
import frc.robot.commands.L3Coral;
import frc.robot.commands.L4Coral;
import frc.robot.commands.L4ElevatorPosition;
import frc.robot.commands.MeetInTheMiddle;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.RumbleRumble;
import frc.robot.commands.SetClimberPosition;
import frc.robot.commands.SetClimberSpeed;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetElevatorPositionInstant;
import frc.robot.commands.ElevatorResetPosition;
import frc.robot.commands.SetElevatorSpeed;
import frc.robot.commands.SetIntakeSpeed;
import frc.robot.commands.SetIntakeSpeedWait;
import frc.robot.commands.SourceIntake;
import frc.robot.commands.MoveSwerve;
import frc.robot.commands.TestCommand;
import frc.robot.commands.WaitForAlign;
import frc.robot.commands.WaitUntilPositionReached;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
	private ElevatorSubsystem elevator;
	private enum Autos {
		Nothing, Three, SILLY6, MeetMeInTheMiddle
	}
	private SendableChooser<Autos> newautopick;
	private Command autoCommand;
	// speed
	private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
	// second
	private Trigger elevatorAndSwerve = new Trigger(() -> (elevator.getPos() > Constants.ElevatorConstants.floorPosition && Robot.getState() == RobotState.TELEOP));
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

	
	/* Setting up bindings for necessary control of the swerve drive platform */
	private final SwerveRequest.RobotCentric limelightSwerve = new SwerveRequest.RobotCentric()
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
		NamedCommands.registerCommand("L2Pos", new SetElevatorPositionInstant(Constants.ElevatorConstants.l2Position));
		NamedCommands.registerCommand("L3", new L3Coral());
		NamedCommands.registerCommand("L4", new L4Coral());
		NamedCommands.registerCommand("StopSwerve", new MoveSwerve(0, 0, 0));
		
		NamedCommands.registerCommand("LimelightLeft", 
			new SequentialCommandGroup(
				new ParallelDeadlineGroup(
					new ParallelRaceGroup(
						new WaitForAlign(false),
						new WaitCommand(10)
					),
					drivetrain.applyRequest(() -> limelightSwerve
							.withVelocityX(LimelightHelpers.getLz() * MaxSpeed * Constants.SwerveConstants.LimelightMultiplier)
							.withVelocityY(LimelightHelpers.getLx() * MaxSpeed * Constants.SwerveConstants.LimelightMultiplier)
							.withRotationalRate(LimelightHelpers.getThetaLeft() * MaxAngularRate * Constants.SwerveConstants.LimelightMultiplier)
					)
				),
				new MoveSwerve(0, 0, 0),
				new ParallelDeadlineGroup(
					new WaitCommand((Constants.SwerveConstants.zPosLeft + Constants.SwerveConstants.swerveError)/(Constants.SwerveConstants.forwardForAuto * MaxSpeed)),
					// new WaitCommand(Constants.SwerveConstants.waitTheyDontLoveYouLikeILoveYou),
					// new WaitUntilPositionReached(Constants.SwerveConstants.zPosLeft + Constants.SwerveConstants.swerveError),
					drivetrain.applyRequest(() -> limelightSwerve
							.withVelocityX(Constants.SwerveConstants.forwardForAuto * MaxSpeed)
							.withVelocityY(0)
							.withRotationalRate(0)
					)
				),
				new MoveSwerve(0, 0, 0)
			)
		);
		NamedCommands.registerCommand("LimelightRight", 
			new SequentialCommandGroup(
				new ParallelDeadlineGroup(
					new ParallelRaceGroup(
						new WaitForAlign(true),
						new WaitCommand(10)
					),
					drivetrain.applyRequest(() -> limelightSwerve
							.withVelocityX(LimelightHelpers.getRz() * MaxSpeed * Constants.SwerveConstants.LimelightMultiplier)
							.withVelocityY(LimelightHelpers.getRx() * MaxSpeed * Constants.SwerveConstants.LimelightMultiplier)
							.withRotationalRate(-LimelightHelpers.getThetaRight() * Constants.SwerveConstants.LimelightMultiplier)
							// .withRotationalRate(-.1)
					)
				),
				new MoveSwerve(0, 0, 0)
			)
		);
		// NamedCommands.registerCommand("Order1", new L1Coral());
		// NamedCommands.registerCommand("Order2", new L2Coral());
		// NamedCommands.registerCommand("Order2", new L2Coral());
		// NamedCommands.registerCommand("Order3", new L3Coral());
		// NamedCommands.registerCommand("Order4", new L4Coral());
		// NamedCommands.registerCommand("SourceIntake", new SourceIntake());
		NamedCommands.registerCommand("StopAndWait", new SequentialCommandGroup(new MoveSwerve(0, 0, 0), new WaitCommand(1)));
		
		newautopick = new SendableChooser<>();
		newautopick.addOption("Nothing", Autos.Nothing);
		newautopick.addOption("Threesome", Autos.Three);
		newautopick.addOption("Silly6", Autos.SILLY6);
		newautopick.addOption("MeetMeInTheMiddle", Autos.MeetMeInTheMiddle);
		Shuffleboard.getTab("auto").add("auto", newautopick).withPosition(0, 0).withSize(3, 1);
		
		// Do this in either robot periodic or subsystem periodic
		field.setRobotPose(drivetrain.getNegativePose());
		configureBindings();
		NamedCommands.registerCommand("L2", new L2Coral());
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

		Inputs.getElevatorSpeedUp().whileTrue(new SetElevatorSpeed(.1));
		Inputs.getElevatorSpeedDown().whileTrue(new SetElevatorSpeed(-.1));
		Inputs.getIntakeIn().whileTrue(new SetIntakeSpeed(-Constants.CoralConstants.intakeSpeed));
		Inputs.getIntakeOut().whileTrue(new SetIntakeSpeed(Constants.CoralConstants.intakeSpeed));

		Inputs.getClimberUp().whileTrue(new SetClimberSpeed(Constants.ClimberConstants.speed));
		Inputs.getClimberDown().whileTrue(new SetClimberSpeed(-Constants.ClimberConstants.speed));
		Inputs.getRUMBLE().whileTrue(new RumbleRumble());
		// Inputs.getTest().onTrue(new DriveToPose(Constants.FieldConstants.coralPose1));
		// Inputs.getTest().onTrue(new SequentialCommandGroup(new MoveSwerve(0, 0, 0), new L2Coral()));
		// Inputs.getTest().onTrue(new CenterAtAprilTag(true));
		// Inputs.getTest().onTrue(new SetIntakeSpeedWait(Constants.CoralConstants.intakeSpeed));
		Inputs.getTest().onTrue(new L2Coral());
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

		//UNCOMMENT THIS PLS PLS PLS DONT PROVE ME RIGHT
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

		
		elevatorAndSwerve.whileTrue(
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
			)
		);
			
		joystick.povLeft().onTrue(
			new SequentialCommandGroup(
				new ParallelDeadlineGroup(
					new ParallelRaceGroup(
						new WaitForAlign(false),
						new WaitCommand(10)
					),
					drivetrain.applyRequest(() -> limelightSwerve
							.withVelocityX(LimelightHelpers.getLz() * MaxSpeed * Constants.SwerveConstants.LimelightMultiplier)
							.withVelocityY(LimelightHelpers.getLx() * MaxSpeed * Constants.SwerveConstants.LimelightMultiplier)
							.withRotationalRate(LimelightHelpers.getThetaLeft() * MaxAngularRate * Constants.SwerveConstants.LimelightMultiplier)
					)
				),
				new MoveSwerve(0, 0, 0),
				new ParallelDeadlineGroup(
					new WaitCommand((Constants.SwerveConstants.zPosLeft + Constants.SwerveConstants.swerveError)/(Constants.SwerveConstants.forwardForAuto * MaxSpeed)),
					// new WaitCommand(Constants.SwerveConstants.waitTheyDontLoveYouLikeILoveYou),
					// new WaitUntilPositionReached(Constants.SwerveConstants.zPosLeft + Constants.SwerveConstants.swerveError),
					drivetrain.applyRequest(() -> limelightSwerve
							.withVelocityX(Constants.SwerveConstants.forwardForAuto * MaxSpeed)
							.withVelocityY(0)
							.withRotationalRate(0)
					)
				),
				new MoveSwerve(0, 0, 0)
			)
		);
		Inputs.getGo().onTrue(
			new SequentialCommandGroup(
				new ParallelDeadlineGroup(
					new WaitCommand((Constants.SwerveConstants.zPosLeft + Constants.SwerveConstants.swerveError)/(Constants.SwerveConstants.forwardForAuto * MaxSpeed)),
					// new WaitCommand(Constants.SwerveConstants.waitTheyDontLoveYouLikeILoveYou),
					// new WaitUntilPositionReached(Constants.SwerveConstants.zPosLeft + Constants.SwerveConstants.swerveError),
					drivetrain.applyRequest(() -> limelightSwerve
							.withVelocityX(Constants.SwerveConstants.forwardForAuto * MaxSpeed)
							.withVelocityY(0)
							.withRotationalRate(0)
					)
				),
				new MoveSwerve(0, 0, 0)
			)
		);
		//auto alignment on driver D-pad:
		joystick.povRight().onTrue(
			new SequentialCommandGroup(
				new ParallelDeadlineGroup(
					new ParallelRaceGroup(
						new WaitForAlign(true),
						new WaitCommand(10)
					),
					drivetrain.applyRequest(() -> limelightSwerve
							.withVelocityX(LimelightHelpers.getRz() * MaxSpeed * Constants.SwerveConstants.LimelightMultiplier)
							.withVelocityY(LimelightHelpers.getRx() * MaxSpeed * Constants.SwerveConstants.LimelightMultiplier)
							.withRotationalRate(-LimelightHelpers.getThetaRight() * Constants.SwerveConstants.LimelightMultiplier)
							// .withRotationalRate(-.1)
					)
				),
				new MoveSwerve(0, 0, 0)
			)
		);
		SmartDashboard.putNumber("ZOffsetRight", 0);
		SmartDashboard.putNumber("constantX", Constants.SwerveConstants.xPosLeft);
		SmartDashboard.putNumber("constantZ", Constants.SwerveConstants.zPosLeft);
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

	private ArrayList<Trajectory> autoTraj = new ArrayList<>();

	public void generateTrajectories(String name){
		try{
			List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(name);
			for(PathPlannerPath path:paths){
				autoTraj.add(TrajectoryGenerator.generateTrajectory(path.getPathPoses(), new TrajectoryConfig(MaxSpeed, MaxAngularRate)));
			// autoTraj.add(path.getPathPoses());
			}
		}
		catch(Exception e){}
	}
	
	public void generateAndLoad() {
		autoCommand = generateAutoCommand();
	}

	private Command generateAutoCommand(){
		switch(newautopick.getSelected()){
			case SILLY6:
			    // start close to curtain (Blue DS right))
				System.out.println("fish1");
				System.out.println("fish1");
				System.out.println("fish1");
				System.out.println("fish1");
				System.out.println("fish1");
				System.out.println("fish1");
				System.out.println("fish1");
				System.out.println("fish1");
				System.out.println("fish1");
				System.out.println("fish1");
				System.out.println("fish1");
				System.out.println("fish1");
				System.out.println("fish1");
				System.out.println("fish1");
				generateTrajectories("TheSilly6WithLimelight");
				return TunerConstants.createDrivetrain().getAuto("TheSilly6WithLimelight");
			case Three:
			//near audience (blue DS left, Red DS right side)
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				System.out.println("fish2");
				generateTrajectories("BottomThreesome");
				return TunerConstants.createDrivetrain().getAuto("BottomThreesome");
			case MeetMeInTheMiddle:
				//middle pillar
				return new SequentialCommandGroup(
					new ParallelDeadlineGroup(
						new ParallelRaceGroup(
							new WaitForAlign(true),
							new WaitCommand(10)
						),
						drivetrain.applyRequest(() -> limelightSwerve
								.withVelocityX(LimelightHelpers.getRz() * MaxSpeed * Constants.SwerveConstants.LimelightMultiplier)
								.withVelocityY(LimelightHelpers.getRx() * MaxSpeed * Constants.SwerveConstants.LimelightMultiplier)
								.withRotationalRate(-LimelightHelpers.getThetaRight() * Constants.SwerveConstants.LimelightMultiplier)
								// .withRotationalRate(-.1)
						)
					),
					new MoveSwerve(0, 0, 0),
					new L4Coral()
				);
			case Nothing:
				return new WaitCommand(1);
			default: //Autos.Nothing
				return new WaitCommand(50000);
		}
	}

	public Command getAutonomousCommand() {
		// return drivetrain.moveTo(new Pose2d(1, 0, new Rotation2d(0)));
		// return new ParallelDeadlineGroup(new WaitCommand(5), new DriveToPose(new Pose2d(0, .5, new Rotation2d(0))));
		// drivetrain.getAuto("TheSilly6");
		// return new WaitCommand(0);
		// PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("TestPath");
		// return drivetrain.getAuto("TheSilly6WithLimelight");
		// return new L2Coral();
		// return new CenterAtAprilTag(true);
		// return new ParallelDeadlineGroup(new WaitCommand(1), new MoveSwerve(1, 0, 0));
		
		// if (autoCommand == null) {
		// 	autoCommand = generateAutoCommand();
		// }
		return generateAutoCommand();
	}
}