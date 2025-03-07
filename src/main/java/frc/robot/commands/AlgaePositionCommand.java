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
public class AlgaePositionCommand extends SequentialCommandGroup {
  boolean auton;

  public enum AlgaeMode {
    PROCESSOR, REEFLOW, REEFHIGH, NET, GROUND;
  }

  public AlgaePositionCommand(AlgaeMode mode, boolean auton) {
    this.auton = auton;

    switch (mode) {
      case PROCESSOR:
        addCommands(
            new ParallelCommandGroup(
                new SetElevatorCommand(ElevatorConstants.processorHeight, auton),
                new SetScorersCommand(ScoringConstants.processorPosition, auton, ScoringMode.ALGAE)));
        break;

      case REEFLOW:
        addCommands(
            new ParallelCommandGroup(
                new SetElevatorCommand(ElevatorConstants.algaeLowHeight, auton),
                new SetScorersCommand(ScoringConstants.algaeLowPosition, auton, ScoringMode.ALGAE)));
        break;

      case REEFHIGH:
        addCommands(
            new ParallelCommandGroup(
                new SetElevatorCommand(ElevatorConstants.algaeHighHeight, auton),
                new SetScorersCommand(ScoringConstants.algaeHighPosition, auton, ScoringMode.ALGAE)));
        break;

      case NET:
        addCommands(
            new ParallelCommandGroup(
                new SetElevatorCommand(ElevatorConstants.netHeight, auton),
                new SetScorersCommand(ScoringConstants.netPosition, auton, ScoringMode.ALGAE)));
        break;

      case GROUND:
        addCommands(
            new ParallelCommandGroup(
                new SetElevatorCommand(ElevatorConstants.groundHeight, auton),
                new SetScorersCommand(ScoringConstants.groundPosition, auton, ScoringMode.ALGAE)));
        break;
    }
  }
}
