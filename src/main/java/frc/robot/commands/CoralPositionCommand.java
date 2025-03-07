// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.*;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.ScoringSubsystem.ScoringMode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralPositionCommand extends SequentialCommandGroup {
  boolean auton;

  public enum CoralMode {
    TROUGH, LOW, MID, HIGH;
  }

  public CoralPositionCommand(CoralMode mode, boolean auton) {
    this.auton = auton;
    switch (mode) {
      case TROUGH:

        addCommands(
            new ParallelCommandGroup(
                new SetElevatorCommand(ElevatorConstants.troughHeight, auton),
                new SetScorersCommand(ScoringConstants.troughPosition, auton, ScoringMode.CORAL)));
        break;

      case LOW:
        addCommands(
            new ParallelCommandGroup(
                new SetElevatorCommand(ElevatorConstants.coralLowHeight, auton),
                new SetScorersCommand(ScoringConstants.coralLowPosition, auton, ScoringMode.CORAL)));
        break;

      case MID:
        addCommands(
            new ParallelCommandGroup(
                new SetElevatorCommand(ElevatorConstants.coralMidHeight, auton),
                new SetScorersCommand(ScoringConstants.coralMidPosition, auton, ScoringMode.CORAL)));
        break;

      case HIGH:
        addCommands(
            new ParallelCommandGroup(
                new SetElevatorCommand(ElevatorConstants.coralHighHeight, auton),
                new SetScorersCommand(ScoringConstants.coralHighPosition, auton, ScoringMode.CORAL)));
        break;
    }
  }

}
