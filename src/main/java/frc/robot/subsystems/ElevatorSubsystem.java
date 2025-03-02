// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax motor;
  private SparkMax motorFollow;
  private double tarPos;
  private SparkMaxConfig config;
  private SparkMaxConfig config2;
  private boolean posControl = false;
  private double ku = 0.08;
  private double t = Robot.kDefaultPeriod;
  private double kP = 0.2;
  private double kD = 0.5;
  private double kI = 0.0;

  private static ElevatorSubsystem instance;

  public static ElevatorSubsystem getInstance() {
    if (instance == null)
      instance = new ElevatorSubsystem();
    return instance;
  }

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    motor = new SparkMax(Constants.MotorIDs.elevator, MotorType.kBrushless);
    motorFollow = new SparkMax(Constants.MotorIDs.elevatorFollow, MotorType.kBrushless);
    tarPos = Constants.ElevatorConstants.floorPosition;
    posControl = false;
    motor.getEncoder().setPosition(0);

    config = new SparkMaxConfig();
    config.inverted(false).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);
    config.smartCurrentLimit(60);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // config2 = config;
    config2 = new SparkMaxConfig();
    config2.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    config2.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);
    config2.follow(motor, true);
    config.smartCurrentLimit(60);
    motorFollow.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setTarPos(double pos) {
    setToBrake();
    posControl = true;
    tarPos = pos;
  }

  public void changePosition(double changePos) {
    posControl = true;
    tarPos += changePos;
  }

  public double getPos() {
    return motor.getEncoder().getPosition();
  }

  public void setSpeed(double speed) {
    if (speed == 0) {
      posControl = true;
      tarPos = motor.getEncoder().getPosition();
    } else {
      posControl = false;
    }
    motor.set(speed);
  }

  public boolean low = false;
  public boolean bottomed = false;
  public double maxcur = 0.0;

  public void ending() {
    tarPos = 0;
    if (!low) {
      // setSpeedWithoutPID(0);
      setToCoast();
      // motor.configure(config, null, null);
      System.out.println("bottoming");
      SmartDashboard.putString("state", "bottoming");
      low = true;
      bottomed = false;
    }
    if (getPos() > 0.1 && getCurrent() < Constants.ElevatorConstants.stallDetectCur) {
      // double cur = getCurrent();
      // if (cur > maxcur) {
      // maxcur = cur;
      // SmartDashboard.putNumber("mcur", cur);
      // }
      // if (cur > Constants.ElevatorConstants.stallDetectCur) {
      // SmartDashboard.putString("state", "bottomed");
      // // motor.set(0);
      // setSpeedWithoutPID(0);
      // bottomed = true;
      // }

      // if (motor.getEncoder().getPosition() > 0.1) {
      setSpeedWithoutPID(-0.1);
      // motorFollow.setSpeedWithoutPID(-0.1);
      SmartDashboard.putString("state", "almost there");
    } else {
      SmartDashboard.putString("state", "bottomed");
      // motor.set(0);
      setSpeedWithoutPID(0);
      bottomed = true;
    }
  }

  public void setSpeedWithoutPID(double speed) {
    posControl = false;
    motor.set(speed);
  }

  public boolean isAtPosition() {
    double dif = getPos() - tarPos;
    // SmartDashboard.putNumber("dif", dif);
    // System.out.println(Math.abs(dif) < Constants.ElevatorConstants.error);
    return Math.abs(dif) < Constants.ElevatorConstants.error;
    // if ((getPosition() >= (position - Constants.ElevatorConstants.error))
    // && (getPosition() <= (position + Constants.ElevatorConstants.error)))
    // return true;
    // return false;
  }

  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  void setBC(boolean coolBool) {
    if (coolBool) {
      config.idleMode(IdleMode.kBrake);
      config2.idleMode(IdleMode.kBrake);
    } else {
      config.idleMode(IdleMode.kCoast);
      config2.idleMode(IdleMode.kCoast);
    }
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorFollow.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setToBrake() {
    setBC(true);
  }

  public void setToCoast() {
    setBC(false);
    // config.idleMode(IdleMode.kCoast);
    // config2.idleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // System.out.print(motor.getEncoder().getPosition());
    // System.out.print(" \t");
    // System.out.println(motor.getOutputCurrent());

    double dif = getPos() - tarPos;
    SmartDashboard.putNumber("dif", dif);
    SmartDashboard.putNumber("cur", getCurrent());
    SmartDashboard.putNumber("positionLol", getPos());
    if (posControl && tarPos >= Constants.ElevatorConstants.floorPosition
        && tarPos < Constants.ElevatorConstants.maxPosition) {
      motor.getClosedLoopController().setReference(tarPos, ControlType.kPosition);
    }
    
  }

}