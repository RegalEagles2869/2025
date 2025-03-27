// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntakeSubsystem extends SubsystemBase {
  private TalonFX motor;

  private TalonFXConfiguration config;
  private double kP = 1;
  private double kD = 0;
  private double currentLimit = 40;
  private final PositionVoltage request = new PositionVoltage(0).withSlot(0);
  
  private static CoralIntakeSubsystem instance;
  
  /** Creates a new CoralIntakeSubsystem. */
  public static CoralIntakeSubsystem getInstance() {
    if (instance == null) instance = new CoralIntakeSubsystem();
    return instance;
  }

  public CoralIntakeSubsystem() {
    motor = new TalonFX(Constants.MotorIDs.coralIntake);

    config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.PeakForwardDutyCycle = 1;
    config.MotorOutput.PeakReverseDutyCycle = -1;
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
    config.Slot0.kP = kP;
    config.Slot0.kI = 0;
    config.Slot0.kD = kD;
    config.Slot0.kG = 0;
    motor.getConfigurator().apply(config);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public double getCurrent() {
    return motor.getSupplyCurrent().getValueAsDouble();
  }

  public void setPosition() {
    motor.setPosition(0);
    motor.setControl(request.withPosition(Constants.CoralConstants.deltaMove));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current intake", getCurrent());
  }
}
