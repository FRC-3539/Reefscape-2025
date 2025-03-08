// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.EnumConstants.*;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.constants.EnumConstants.*;


public class SetScorersCommand extends Command {
	/** Creates a new SetElevatorCommand. */
	double angle;
	boolean wait;
	ScoringMode mode;

	/** Creates a new AngleShooterCommand. */
	public SetScorersCommand(double angle, boolean wait, ScoringMode mode) {
		// Use addRequirements() here to declare subsystem dependencies..a
		this.angle = angle;
		this.wait = wait;
		this.mode = mode;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		ScoringSubsystem.setRotateAngle(angle);
		ScoringSubsystem.setIntakeMode(mode);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// System.out.println(height + " " +
		if (!wait) {
			return true;
		} else {
			return MathUtil.isNear(angle, ScoringSubsystem.getRotateAngle(), 1);
		}
	}
}
