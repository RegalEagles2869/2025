// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax motor;
  private SparkMax motorFollow;
  private double position;
  private SparkMaxConfig config;
  private SparkMaxConfig config2;
  private boolean posControl = false;

  private static ElevatorSubsystem instance;

  public static ElevatorSubsystem getInstance() {
    if (instance == null) instance = new ElevatorSubsystem();
    return instance;
  }
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    motor = new SparkMax(Constants.MotorIDs.elevator, MotorType.kBrushless);
    motorFollow = new SparkMax(Constants.MotorIDs.elevatorFollow, MotorType.kBrushless);
    position = Constants.ElevatorConstants.floorPosition;
    posControl = false;

    config = new SparkMaxConfig();
    config.inverted(false).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config2 = new SparkMaxConfig();
    config2.inverted(true).idleMode(IdleMode.kBrake);
    config2.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    config2.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);
    config2.follow(Constants.MotorIDs.elevator);
    motorFollow.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setPosition(double pos) {
    posControl = true;
    position = pos;
  }

  public void changePosition(double changePos) {
    posControl = true;
    position += changePos;
  }

  public double getPosition() {
    return motor.getAbsoluteEncoder().getPosition();
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public boolean isAtPosition() {
    if ((getPosition() >= (position - Constants.ElevatorConstants.error)) && (getPosition() <= (position + Constants.ElevatorConstants.error)))
      return true;
    return false;
  }

  @Override
  public void periodic() {
    if (posControl) {
      if (position >= Constants.ElevatorConstants.floorPosition && position < Constants.ElevatorConstants.maxPosition) {
        motor.getClosedLoopController().setReference(position, ControlType.kPosition);
        if (isAtPosition()) posControl = false;
      }
    }
  }
}
