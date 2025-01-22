// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MotorConfiguration;

public class CoralPivotSubsystem extends SubsystemBase {

  private TalonFX motor;
  private double position;
  
  private static CoralPivotSubsystem instance;

  public static CoralPivotSubsystem getInstance() {
    if (instance == null) instance = new CoralPivotSubsystem();
    return instance;
  }
  /** Creates a new CoralPivotSubsystem. */
  public CoralPivotSubsystem() {
    motor = new TalonFX(Constants.MotorIDs.CoralPivot);
    MotorConfiguration.configureMotor(motor, Constants.PivotConstants.config);
  }

  public void setPosition(double pos) {
    position = pos;
  }

  public void changePosition(double changePos) {
    position += changePos;
  }

  public boolean isAtPosition() {
    return false;
  }

  @Override
  public void periodic() {
    if (position >= Constants.PivotConstants.floorPosition && position < Constants.PivotConstants.maxPosition) {
      motor.setPosition(position);
    }
  }
}
