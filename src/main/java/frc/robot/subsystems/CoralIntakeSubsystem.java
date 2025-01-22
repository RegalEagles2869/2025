// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MotorConfiguration;

public class CoralIntakeSubsystem extends SubsystemBase {
  /** Creates a new CoralIntakeSubsystem. */
  private TalonFX motor;

  private static CoralIntakeSubsystem instance;

  public static CoralIntakeSubsystem getInstance() {
    if (instance == null) instance = new CoralIntakeSubsystem();
    return instance;
  }

  public CoralIntakeSubsystem() {
    motor = new TalonFX(Constants.MotorIDs.coralIntake);
    MotorConfiguration.configureMotor(motor, Constants.CoralConstants.config);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
