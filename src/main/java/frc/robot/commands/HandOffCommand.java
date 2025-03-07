// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetFunnelPositionCommand.IntakeMode;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ScoringConstants;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.ScoringSubsystem.ScoringMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HandOffCommand extends SequentialCommandGroup {
  boolean auton;
  IntakeMode mode;

  public HandOffCommand(boolean auton, IntakeMode mode) {
    ScoringSubsystem.setIntakeMode(ScoringSubsystem.ScoringMode.CORAL);
    this.auton = auton;
    this.mode = mode;

    addCommands(
      new ParallelCommandGroup(
        new SetElevatorCommand(ElevatorConstants.handOffHeight, auton),
        new SetScorersCommand(ScoringConstants.handOffPosition, auton, ScoringMode.CORAL),
        new SetFunnelPositionCommand(mode)));
  }
}
