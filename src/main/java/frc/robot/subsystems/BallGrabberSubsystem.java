// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallGrabberSubsystem extends SubsystemBase {
  private SparkMax motor1;
  private SparkMax motor2;

  private static BallGrabberSubsystem instance;

  public static BallGrabberSubsystem getInstance() {
    if (instance == null) instance = new BallGrabberSubsystem();
    return instance;
  }

  /** Creates a new BallGrabberSubsystem. */
  public BallGrabberSubsystem() {
    motor1 = new SparkMax(Constants.MotorIDs.ballGrabber1, MotorType.kBrushless);
    motor2 = new SparkMax(Constants.MotorIDs.ballGrabber2, MotorType.kBrushless);
  }

  public void setSpeed(double speed) {
    // motor1.set(speed);
    // motor2.set(-speed);
  }

  @Override
  public void periodic() {
  }
}
