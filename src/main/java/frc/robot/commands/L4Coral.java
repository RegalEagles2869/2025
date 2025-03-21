// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L4Coral extends SequentialCommandGroup {
  /** Creates a new L1Coral. */
  public L4Coral() {
    addCommands(
      // new SetElevatorPositionInstant(Constants.ElevatorConstants.l2Position),
      new SetElevatorPosition(Constants.ElevatorConstants.l4Position),
      new WaitCommand(.1),
      new ParallelDeadlineGroup(
      //   // new WaitCommand(Constants.CoralConstants.intakeTime), 
        new WaitCommand(.1), 
        new SetIntakeSpeed(Constants.CoralConstants.intakeSpeed)
      ),
      new ElevatorToFloor()
    );
  }
}
