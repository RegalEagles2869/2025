// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MotorConfiguration;

public class ElevatorSubsystem extends SubsystemBase {

  private TalonFX motor;
  private double position;

  private static ElevatorSubsystem instance;

  public static ElevatorSubsystem getInstance() {
    if (instance == null) instance = new ElevatorSubsystem();
    return instance;
  }
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    motor = new TalonFX(Constants.MotorIDs.elevator);
    MotorConfiguration.configureMotor(motor, Constants.ElevatorConstants.config);
    position = Constants.ElevatorConstants.floorPosition;
  }

  public void setPosition(double pos) {
    position = pos;
  }

  public void changePosition(double changePos) {
    position += changePos;
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public boolean isAtPosition() {
    if ((getPosition() >= (position - Constants.ElevatorConstants.error)) && (getPosition() <= (position + Constants.ElevatorConstants.error)))
      return true;
    return false;
  }

  @Override
  public void periodic() {
    if (position >= Constants.ElevatorConstants.floorPosition && position < Constants.ElevatorConstants.maxPosition) {
      motor.setPosition(position);
    }
  }
}
