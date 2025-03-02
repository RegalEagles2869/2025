// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Time;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static double speedMultiplier = 1;
		public static final double speedMultiplierElev = 0.2;
		public static final double speedMultiplierSlowMode = .1;
		public static final double driveDeadBand = .1;
		public static final double rotationDeadBand = .1;
	}

	public static class FieldConstants {

	}

	public static class MotorIDs {
		public static final int ballGrabber1 = 0;
		public static final int ballGrabber2 = 0;
		public static final int elevator = 6;
		public static final int elevatorFollow = 7;
		public static final int climberID = 15;
		public static final int coralIntake = 14;
		public static final int coralPivot = 15;
	}

	public static class ElevatorConstants {
		public static final double floorPosition = 2;
		public static final double maxPosition = 10000;
		public static final double basePosition = 0;
		public static final double l1Position = 0;
		public static final double l2Position = 13.690503120422363;
		public static final double l3Position = 35.83300018310547;
		// public static final double l4Position = 70.19239807128906;
		public static final double l4Position = 70.19239807128906;
		public static final double l4Position2 = l4Position + 1;
		public static final double sourcePosition = 0;
		public static final double error = 0.2;
		public static final double stallDetectCur = 40.0;
		public static final double slowPose = 5.0;
	}

	public static class PivotConstants {
		public static final double floorPosition = 0;
		public static final double maxPosition = 10000;

		public static final double basePosition = 0;
		public static final double l1Position = 0;
		public static final double l2Position = 0;
		public static final double l3Position = 0;
		public static final double l4Position = 0;
		public static final double sourcePosition = 0;
		public static final double error = 0;
	}

	public static class BallGrabberConstants {
	}

	public static class ClimberConstants {

		public static final double floorPosition = 0;
		public static final double goodPosition = 250;
		public static final double maxPosition = 100000;
		public static final double error = 0;
	}

	public static class CoralConstants {

		public static final double minCurrentRunning = 0;
		public static final double intakeSpeed = 1.0;
		public static final double intakeTime = .5;
	}
}
