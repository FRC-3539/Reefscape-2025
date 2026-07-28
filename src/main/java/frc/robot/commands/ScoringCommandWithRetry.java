// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ScoringSubsystem;

/**
 * Drop-in replacement for the "ScoringCommand" named command that verifies the coral actually
 * left the effector (ScoringSubsystem.coralDetected()) instead of just running for a fixed dwell,
 * and retries up to MAX_ATTEMPTS if it didn't. Skips entirely if the preceding intake never
 * confirmed a coral (ScoringSubsystem.lastIntakeConfirmedCoral), since there's nothing to score.
 */
public class ScoringCommandWithRetry extends SequentialCommandGroup {
  private static final int MAX_ATTEMPTS = 2;
  private static final double FIRST_DWELL_S = 0.35; // matches the dwell it replaces
  private static final double RETRY_DWELL_S = 0.3;
  private static final double SETTLE_S = 0.1;

  public ScoringCommandWithRetry() {
    addCommands(Commands.either(
        attempt(MAX_ATTEMPTS),
        Commands.runOnce(() -> log("no confirmed coral -- skipping score")),
        () -> ScoringSubsystem.lastIntakeConfirmedCoral));
  }

  private static Command attempt(int attemptsLeft) {
    double dwell = attemptsLeft == MAX_ATTEMPTS ? FIRST_DWELL_S : RETRY_DWELL_S;
    return Commands.defer(() -> Commands.sequence(
        new ScoringCommand(false).withTimeout(dwell),
        Commands.waitSeconds(SETTLE_S),
        Commands.either(
            Commands.runOnce(() -> log("scored, coral clear")),
            attemptsLeft > 1 ? attempt(attemptsLeft - 1) : Commands.runOnce(() -> log("score retries exhausted")),
            () -> !ScoringSubsystem.coralDetected())),
        Set.of(RobotContainer.ScoringSubsystem));
  }

  private static void log(String message) {
    SmartDashboard.putString("/ScoringCommandWithRetry/LastEvent", message);
    DriverStation.reportWarning("[ScoringCommandWithRetry] " + message, false);
  }
}
