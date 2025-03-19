// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ScoringConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.constants.EnumConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetFunnelPositionCommand extends Command {

  IntakeMode mode;

  /** Creates a new SetFunnelPositionCommand. */
  public SetFunnelPositionCommand(IntakeMode mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mode = mode;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (mode) {
      case GROUND:
        LedSubsystem.setIntaking(true);
        IntakeSubsystem.setFunnelDeployAngle(IntakeConstants.groundFunnelDeployAngle);
        IntakeSubsystem.setCoralIntakeMotor(IntakeConstants.coralIntakeVoltage);
        IntakeSubsystem.setFunnelIntakeMotor(0);
        break;

      case HUMAN:
        LedSubsystem.setIntaking(true);
        IntakeSubsystem.setFunnelDeployAngle(IntakeConstants.humanFunnelDeployAngle);
        IntakeSubsystem.setCoralIntakeMotor(IntakeConstants.coralIntakeVoltage);
        IntakeSubsystem.setFunnelIntakeMotor(0);
        break;

      case REVERSE:
        LedSubsystem.setIntaking(true);
        IntakeSubsystem.setFunnelDeployAngle(0);
        IntakeSubsystem.setCoralIntakeMotor(-IntakeConstants.coralIntakeVoltage);
        IntakeSubsystem.setFunnelIntakeMotor(-IntakeConstants.funnelIntakeVoltage);
        break;

      case HOME:
        IntakeSubsystem.setFunnelDeployAngle(IntakeConstants.homeFunnelDeployAngle);
        IntakeSubsystem.setCoralIntakeMotor(0);
        IntakeSubsystem.setFunnelIntakeMotor(0);
        break;

      case HANDOFF:
        IntakeSubsystem.setFunnelDeployAngle(IntakeConstants.handOffFunnelDeployAngle);
        IntakeSubsystem.setCoralIntakeMotor(IntakeConstants.coralIntakeVoltage);
        IntakeSubsystem.setFunnelIntakeMotor(IntakeConstants.funnelIntakeVoltage);
        break;

      case CLIMB:
        IntakeSubsystem.setFunnelDeployAngle(IntakeConstants.groundFunnelDeployAngle);
        IntakeSubsystem.setCoralIntakeMotor(0);
        IntakeSubsystem.setFunnelIntakeMotor(0);
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (mode == IntakeMode.REVERSE || mode == IntakeMode.CLIMB) {
      return;
    }

    if (MathUtil.isNear(ScoringConstants.handOffPosition, ScoringSubsystem.getRotateAngle(), 5)
        && MathUtil.isNear(IntakeConstants.handOffFunnelDeployAngle, IntakeSubsystem.getFunnelDeployAngle(), 5)) {
      if (ScoringSubsystem.coralDetected()) {
        IntakeSubsystem.setFunnelIntakeMotor(0);
        ScoringSubsystem.scoringMotor(0);
        IntakeSubsystem.setCoralIntakeMotor(0);
      } else {
        IntakeSubsystem.setFunnelIntakeMotor(IntakeConstants.funnelIntakeVoltage);
        ScoringSubsystem.scoringMotor(ScoringConstants.algaeScoringVoltage);
        IntakeSubsystem.setCoralIntakeMotor(IntakeConstants.coralIntakeVoltage);
      }

    } else {
      IntakeSubsystem.setFunnelIntakeMotor(0);
      ScoringSubsystem.scoringMotor(0);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // IntakeSubsystem.setFunnelDeployAngle(IntakeConstants.homeFunnelDeployAngle);
    IntakeSubsystem.setCoralIntakeMotor(0);
    IntakeSubsystem.setFunnelIntakeMotor(0);
    ScoringSubsystem.scoringMotor(0);
    LedSubsystem.setIntaking(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (mode == IntakeMode.CLIMB) {
      return true;
    }
    return ScoringSubsystem.coralDetected()
        && MathUtil.isNear(ScoringSubsystem.getRotateAngle(), ScoringConstants.handOffPosition, 10);
  }
}
