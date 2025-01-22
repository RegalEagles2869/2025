// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MotorConfiguration;

public class ClimberSubsystem extends SubsystemBase {

  private TalonFX motor;

  private static ClimberSubsystem instance;

  public static ClimberSubsystem getInstance() {
    if (instance == null) instance = new ClimberSubsystem();
    return instance;
  }

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    motor = new TalonFX(Constants.MotorIDs.climberID);
    MotorConfiguration.configureMotor(motor, Constants.ClimberConstants.config);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public void climb() {
    motor.set(1);
  }

  @Override
  public void periodic() {
    
  }
}
