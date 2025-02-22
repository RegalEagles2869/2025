// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntakeSubsystem extends SubsystemBase {
  /** Creates a new CoralIntakeSubsystem. */
  private SparkMax motor;

  private static CoralIntakeSubsystem instance;

  public static CoralIntakeSubsystem getInstance() {
    if (instance == null) instance = new CoralIntakeSubsystem();
    return instance;
  }

  public CoralIntakeSubsystem() {
    motor = new SparkMax(Constants.MotorIDs.coralIntake, MotorType.kBrushless);
  }

  public void setSpeed(double speed) {
    System.out.println(speed);
    motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
