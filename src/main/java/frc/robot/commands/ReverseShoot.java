// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReverseShoot extends Command {
  /** Creates a new ReverseShoot. */
  private boolean algae;
  public ReverseShoot(boolean algae) {
    this.algae = algae;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  public ReverseShoot() {
    this.algae = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (algae) ScoringSubsystem.setShootingMotor(-8);
    else ScoringSubsystem.setShootingMotor(-4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ScoringSubsystem.setShootingMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!algae) return ScoringSubsystem.getCoralDistance() < 0.15;
    else return false;
  }
}
