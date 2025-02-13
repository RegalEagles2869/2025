// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private SparkMax motor;

  private static ClimberSubsystem instance;

  public static ClimberSubsystem getInstance() {
    if (instance == null) instance = new ClimberSubsystem();
    return instance;
  }

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    motor = new SparkMax(Constants.MotorIDs.climberID, MotorType.kBrushless);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }


  @Override
  public void periodic() {
    
  }
}
