// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {

  private SparkFlex motor;
  private double position;

  private SparkMaxConfig config;
  
  private static PivotSubsystem instance;

  private boolean posControl = false;

  public static PivotSubsystem getInstance() {
    if (instance == null) instance = new PivotSubsystem();
    return instance;
  }
  /** Creates a new CoralPivotSubsystem. */
  public PivotSubsystem() {
    motor = new SparkFlex(Constants.MotorIDs.coralPivot, MotorType.kBrushless);
    posControl = false;

    config = new SparkMaxConfig();
    config.inverted(false).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(.1, 0.0, 0.0);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setPosition(double pos) {
    posControl = true;
    position = pos;
  }

  public void changePosition(double changePos) {
    posControl = true;
    position += changePos;
  }

  public void setSpeed(double speed) {
    if (speed == 0) {
      posControl = true;
      position = motor.getEncoder().getPosition();
    }
    else posControl = false;
    motor.set(speed);
  }

  public double getPosition() {
    return motor.getEncoder().getPosition();
  }

  public boolean isAtPosition() {
    if ((getPosition() >= (position - Constants.PivotConstants.error)) && (getPosition() <= (position + Constants.PivotConstants.error)))
      return true;
    return false;
  }

  @Override
  public void periodic() {
    if (posControl) {
      if (position >= Constants.PivotConstants.floorPosition && position < Constants.PivotConstants.maxPosition) {
        System.out.println(position);
        motor.getClosedLoopController().setReference(position, ControlType.kPosition);
      }
    }
  }
}
