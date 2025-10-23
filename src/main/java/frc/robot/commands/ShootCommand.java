// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ScoringConstants;
import frc.robot.constants.EnumConstants.ScoringMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.LedSubsystem.LEDState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  private boolean algae;
  /** Creates a new ShootCommand. */
  public ShootCommand() {
    this.algae = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ShootCommand(boolean algae) {
    this.algae = algae;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(
  ) {
    //  
    if (IntakeSubsystem.getMode() == ScoringMode.CORAL) ScoringSubsystem.setShootingMotor(8);

    else ScoringSubsystem.setShootingMotor(-8);
     LedSubsystem.setLEDs(LEDState.SHOOTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (IntakeSubsystem.getMode() == ScoringMode.ALGAE) return;
    if (DriverStation.isAutonomous() || RobotContainer.leftScriptCommand.isScheduled()) return;
    double multiplier = RobotContainer.operatorController.getRightTriggerAxis();
    ScoringSubsystem.setShootingMotor(8*multiplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ScoringSubsystem.setShootingMotor(0);
    LedSubsystem.setLEDs(LEDState.ON);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
