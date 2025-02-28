// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private TalonFX motor;
  private TalonFXConfiguration config;
  private double position;
  private double kP = 5;
  private double kD = 0;
  private double currentLimit = 40;
  private final PositionVoltage request = new PositionVoltage(0).withSlot(0);
  
  private static ClimberSubsystem instance;

  private boolean posControl = false;

  public static ClimberSubsystem getInstance() {
    if (instance == null) instance = new ClimberSubsystem();
    return instance;
  }
  /** Creates a new CoralPivotSubsystem. */
  public ClimberSubsystem() {
    motor = new TalonFX(Constants.MotorIDs.climberID);
    posControl = false;

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
      position = getPosition();
    }
    else posControl = false;
    motor.set(speed);
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public boolean isAtPosition() {
    if ((getPosition() >= (position - Constants.ClimberConstants.error)) && (getPosition() <= (position + Constants.ClimberConstants.error)))
      return true;
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ClimberPos", getPosition());
    if (posControl) {
      if (position >= Constants.ClimberConstants.floorPosition && position < Constants.ClimberConstants.maxPosition) {
        motor.setControl(request.withPosition(position));
      }
    }
  }
}