// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetFunnelPositionCommand extends Command {
  public enum IntakeMode {
		GROUND, HUMAN, REVERSE, HOME;
	}  

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
			case GROUND :
        IntakeSubsystem.setFunnelDeployAngle(IntakeConstants.groundFunnelDeployAngle);
        IntakeSubsystem.setCoralIntakeMotor(IntakeConstants.coralIntakeVoltage);
        IntakeSubsystem.setFunnelIntakeMotor(IntakeConstants.funnelIntakeVoltage);

      break;
    
      case HUMAN :
        IntakeSubsystem.setFunnelDeployAngle(IntakeConstants.humanFunnelDeployAngle);
        IntakeSubsystem.setCoralIntakeMotor(IntakeConstants.coralIntakeVoltage);
        IntakeSubsystem.setFunnelIntakeMotor(IntakeConstants.funnelIntakeVoltage);

      break;

      case REVERSE :
        IntakeSubsystem.setFunnelDeployAngle(IntakeConstants.groundFunnelDeployAngle);
        IntakeSubsystem.setCoralIntakeMotor(-IntakeConstants.coralIntakeVoltage);
        IntakeSubsystem.setFunnelIntakeMotor(-IntakeConstants.funnelIntakeVoltage);

      break;
      case HOME :
      IntakeSubsystem.setFunnelDeployAngle(IntakeConstants.homeFunnelDeployAngle);
      IntakeSubsystem.setCoralIntakeMotor(0);
      IntakeSubsystem.setFunnelIntakeMotor(0);

    break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.setFunnelDeployAngle(IntakeConstants.homeFunnelDeployAngle);
    IntakeSubsystem.setCoralIntakeMotor(0);
    IntakeSubsystem.setFunnelIntakeMotor(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
