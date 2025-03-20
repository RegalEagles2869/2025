// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntakeSubsystem extends SubsystemBase {
  private TalonFX motor;
  
  private static CoralIntakeSubsystem instance;
  
  /** Creates a new CoralIntakeSubsystem. */
  public static CoralIntakeSubsystem getInstance() {
    if (instance == null) instance = new CoralIntakeSubsystem();
    return instance;
  }

  public CoralIntakeSubsystem() {
    motor = new TalonFX(Constants.MotorIDs.coralIntake);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public double getCurrent() {
    return motor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current intake", getCurrent());
  }
}
