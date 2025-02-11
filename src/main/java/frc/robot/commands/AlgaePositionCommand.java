// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAlgaePositionCommand extends SequentialCommandGroup {
  public enum AlgaeMode {
		PROCESSOR, REEFLOW, REEFHIGH, NET;
	}  

public SetAlgaePositionCommand(AlgaeMode mode) {
    switch (mode) {
			case PROCESSOR :

      addCommands(
        new ParallelCommandGroup(
          new SetElevatorCommand(ElevatorConstants.processorHeight), 
          new SetScorersCommand(ScoringConstants.processorPosition)));
      break;

			case REEFLOW :
			addCommands(
        new ParallelCommandGroup(
          new SetElevatorCommand(ElevatorConstants.reefLowHeight), 
          new SetScorersCommand(ScoringConstants.reefLowPosition)));
      break;

			case REEFHIGH :
      addCommands(
        new ParallelCommandGroup(
          new SetElevatorCommand(ElevatorConstants.reefHighHeight), 
          new SetScorersCommand(ScoringConstants.reefHighPosition)));
				break;

      case NET: 
      addCommands(
        new ParallelCommandGroup(
          new SetElevatorCommand(ElevatorConstants.netHeight), 
          new SetScorersCommand(ScoringConstants.netPosition)));
      break;  
    }
  }
}

