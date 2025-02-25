// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 0;
    public static final double speedMultiplier = 1;
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
    public static final int climberID = 14;
    public static final int coralIntake = 5;
    public static final int coralPivot = 15;
  }
  public static class ElevatorConstants {
    public static final double floorPosition = 0;
    public static final double maxPosition = 10000;
    // config.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    // config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);

    public static final double basePosition = 0;
    public static final double l1Position = 0;
    public static final double l2Position = 0;
    public static final double l3Position = 0;
    public static final double l4Position = 0;
    public static final double sourcePosition = 0;
    public static final double error = 0;
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
    public static final double goodPosition = 150;
    public static final double maxPosition = 100000;
    public static final double error = 0;
  }
  public static class CoralConstants {

    public static final int minCurrentRunning = 0;
    public static final int intakeSpeed = 1;
  }  
}
