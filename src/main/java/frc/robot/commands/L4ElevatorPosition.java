// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L4ElevatorPosition extends SequentialCommandGroup {
  /** Creates a new L4ElevatorPosition. */
  public L4ElevatorPosition() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetElevatorPositionInstant(Constants.ElevatorConstants.l4Position),
        new ParallelCommandGroup(new ParallelRaceGroup(new SetIntakeSpeed(.1), new WaitCommand(2)),
            new SetElevatorPositionInstant(Constants.ElevatorConstants.l4Position2)));
  }
}
