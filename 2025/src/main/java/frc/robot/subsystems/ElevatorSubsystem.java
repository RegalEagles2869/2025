// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  
  private TalonFX motor;
  private double position;

  private static ElevatorSubsystem elevator;

  public static ElevatorSubsystem getInstance() {
    if (elevator == null) elevator = new ElevatorSubsystem();
    return elevator;
  }
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    motor = new TalonFX(Constants.MotorIDs.elevator);
    position = Constants.ElevatorConstants.floorPosition;
  }

  public void setPosition(double pos) {
    position = pos;
  }

  @Override
  public void periodic() {
    if (position >= Constants.ElevatorConstants.floorPosition && position < Constants.ElevatorConstants.maxPosition) {
      motor.setPosition(position);
    }
  }
}
