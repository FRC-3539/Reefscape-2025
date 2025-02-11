// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.*;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralPositionCommand extends SequentialCommandGroup {
  public enum CoralMode {
		TROUGH, LOW, MID, HIGH;
	}   
  public CoralPositionCommand(CoralMode mode) {
    switch (mode) {
			case TROUGH :

      addCommands(
        new ParallelCommandGroup(
          new SetElevatorCommand(ElevatorConstants.troughHeight), 
          new SetScorersCommand(ScoringConstants.troughPosition)));
      break;

			case LOW :
			addCommands(
        new ParallelCommandGroup(
          new SetElevatorCommand(ElevatorConstants.lowHeight), 
          new SetScorersCommand(ScoringConstants.lowPosition)));
      break;

			case MID :
      addCommands(
        new ParallelCommandGroup(
          new SetElevatorCommand(ElevatorConstants.midHeight), 
          new SetScorersCommand(ScoringConstants.midPosition)));
				break;

      case HIGH: 
      addCommands(
        new ParallelCommandGroup(
          new SetElevatorCommand(ElevatorConstants.highHeight), 
          new SetScorersCommand(ScoringConstants.highPosition)));
      break;
		}
  }

}
