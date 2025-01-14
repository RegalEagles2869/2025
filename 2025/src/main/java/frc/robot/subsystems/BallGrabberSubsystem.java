// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MotorConfiguration;

public class BallGrabberSubsystem extends SubsystemBase {
  private TalonFX motor1;
  private TalonFX motor2;

  private static BallGrabberSubsystem instance;

  public static BallGrabberSubsystem getInstance() {
    if (instance == null) instance = new BallGrabberSubsystem();
    return instance;
  }

  /** Creates a new BallGrabberSubsystem. */
  public BallGrabberSubsystem() {
    motor1 = new TalonFX(Constants.MotorIDs.ballGrabber1);
    motor2 = new TalonFX(Constants.MotorIDs.ballGrabber1);
    MotorConfiguration.configureMotor(motor1, Constants.BallGrabberConstants.config1);
    MotorConfiguration.configureMotor(motor2, Constants.BallGrabberConstants.config2);
  }

  public void setSpeed(double speed) {
    motor1.set(speed);
    motor2.set(-speed);
  }

  @Override
  public void periodic() {
    

    // This method will be called once per scheduler run
  }
}
