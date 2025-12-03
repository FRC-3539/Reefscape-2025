// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetElevatorCommand extends Command {
	/** Creates a new SetElevatorCommand. */
	double height;
	boolean wait;

	/** Creates a new AngleShooterCommand. */
	public SetElevatorCommand(double height, boolean wait) {
		// Use addRequirements() here to declare subsystem dependencies..a
		addRequirements(RobotContainer.elevatorSubsystem);
		this.height = height;
		this.wait = wait;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		ElevatorSubsystem.setElevatorPosition(height);

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
		if(!wait)
		{
			return true;
		}
		else
		{
			return MathUtil.isNear(height, ElevatorSubsystem.getElevatorPosition(), 1);
		}
		// System.out.println(height + " " +
		// RobotContainer.shooterSubsystem.getElevatorPosition());
	}
}
