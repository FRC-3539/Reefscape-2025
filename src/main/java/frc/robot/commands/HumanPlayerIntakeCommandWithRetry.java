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
import frc.robot.constants.EnumConstants.*;
import frc.robot.subsystems.ScoringSubsystem;

/**
 * Drop-in replacement for the "HumanPlayerIntakeCommand" named command that retries the hand-off
 * if it never confirms a coral, and records the outcome in ScoringSubsystem.lastIntakeConfirmedCoral
 * so the following ScoringCommandWithRetry knows whether there's anything to score. Each attempt
 * races FunnelRangeCommand alongside the hand-off (same early-exit trick the 4-piece autos already
 * use on the funnel-throat sensor) so a good hand-off doesn't wait out the full timeout, but the
 * retry/success decision is still made on the stricter ScoringSubsystem.coralDetected() afterward.
 */
public class HumanPlayerIntakeCommandWithRetry extends SequentialCommandGroup {
  private static final int MAX_ATTEMPTS = 2;
  private static final double FIRST_TIMEOUT_S = 5.0; // matches the timeout it replaces
  private static final double RETRY_TIMEOUT_S = 2.0; // not another blind 5s -- see plan's time-budget note
  private static final double SETTLE_S = 0.15; // lets a funnel-triggered exit finish arriving before checking

  public HumanPlayerIntakeCommandWithRetry() {
    addCommands(attempt(MAX_ATTEMPTS));
  }

  private static Command attempt(int attemptsLeft) {
    double timeout = attemptsLeft == MAX_ATTEMPTS ? FIRST_TIMEOUT_S : RETRY_TIMEOUT_S;
    return Commands.defer(() -> Commands.sequence(
        Commands.race(new HandOffCommand(true, IntakeMode.HUMAN), new FunnelRangeCommand()).withTimeout(timeout),
        Commands.waitSeconds(SETTLE_S),
        Commands.either(
            Commands.runOnce(() -> succeed()),
            attemptsLeft > 1 ? attempt(attemptsLeft - 1) : Commands.runOnce(() -> giveUp()),
            () -> ScoringSubsystem.coralDetected())),
        Set.of(RobotContainer.ScoringSubsystem, RobotContainer.ElevatorSubsystem));
  }

  private static void succeed() {
    ScoringSubsystem.lastIntakeConfirmedCoral = true;
    log("coral confirmed");
  }

  private static void giveUp() {
    ScoringSubsystem.lastIntakeConfirmedCoral = false;
    log("gave up, no coral confirmed");
  }

  private static void log(String message) {
    SmartDashboard.putString("/HumanPlayerIntakeCommandWithRetry/LastEvent", message);
    DriverStation.reportWarning("[HumanPlayerIntakeCommandWithRetry] " + message, false);
  }
}
