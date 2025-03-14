// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.EnumConstants.IntakeMode;
import frc.robot.constants.EnumConstants.ScoringMode;
import frc.robot.constants.ScoringConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbPositionCommand extends SequentialCommandGroup {
  /** Creates a new ClimbPositionCommand. */
  public ClimbPositionCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new SetFunnelPositionCommand(IntakeMode.CLIMB),
            new SetElevatorCommand(ElevatorConstants.coralLowHeight, false),
            new SetScorersCommand(ScoringConstants.coralLowPosition, false, ScoringMode.CORAL)));
  }
}
