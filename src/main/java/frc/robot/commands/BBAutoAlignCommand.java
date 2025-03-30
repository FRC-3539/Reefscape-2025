// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.AlignConstants;
import frc.robot.constants.EnumConstants.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LedSubsystem;

import java.util.ArrayList;
import java.util.List;
import org.frcteam3539.Byte_Swerve_Lib.control.MaxAccelerationConstraint;
import org.frcteam3539.Byte_Swerve_Lib.control.MaxVelocityConstraint;
import org.frcteam3539.Byte_Swerve_Lib.control.SimplePathBuilder;
import org.frcteam3539.Byte_Swerve_Lib.control.Trajectory;
import org.frcteam3539.Byte_Swerve_Lib.control.TrajectoryConstraint;

public class BBAutoAlignCommand extends Command {
	/** Wrapper command to generate a trajectory to the nearest Stage Pose */
	AlignPoint mode;
	ScoringMode piece;

	public BBAutoAlignCommand(AlignPoint mode, ScoringMode piece) {
		this.mode = mode;
		this.piece = piece;
	}

	// Called when the command is initially scheduled.
	@Override

	public void initialize() {
		LedSubsystem.setAligning(true);
		Pose2d targetPoint, straightAlgae = new Pose2d();
		if (mode == AlignPoint.CORALLEFT) {
			targetPoint = RobotContainer.DriveSubsystem.getPose2d()
					.nearest(new ArrayList<Pose2d>(AlignConstants.coralPointsLeft.values()));
		} else if (mode == AlignPoint.CORALRIGHT) {
			targetPoint = RobotContainer.DriveSubsystem.getPose2d()
					.nearest(new ArrayList<Pose2d>(AlignConstants.coralPointsRight.values()));
		} else if (mode == AlignPoint.ALGAE) {
			targetPoint = RobotContainer.DriveSubsystem.getPose2d()
					.nearest(new ArrayList<Pose2d>(AlignConstants.algaePoints.values()));
			// straightAlgae = targetPoint
			// 		.nearest(new ArrayList<Pose2d>(AlignConstants.straightPoints.values()));
		} else if (piece == ScoringMode.CLIMB) {
			targetPoint = RobotContainer.DriveSubsystem.getPose2d()
					.nearest(new ArrayList<Pose2d>(AlignConstants.climbPoints.values()));
		} else {
			targetPoint = RobotContainer.DriveSubsystem.getPose2d()
					.nearest(new ArrayList<Pose2d>(AlignConstants.humanPlayerPoints.values()));
		}

		Pose2d robotPose = RobotContainer.DriveSubsystem.getPose2d();

		// Generate trajectory command to nearest coordinate
		if (mode == AlignPoint.ALGAE) {
			RobotContainer.DriveSubsystem.getFollower()
					.follow(new Trajectory(new SimplePathBuilder(robotPose).lineTo(targetPoint).lineTo(straightAlgae).build(),
							new TrajectoryConstraint[] { (TrajectoryConstraint) new MaxAccelerationConstraint(1.25),
									(TrajectoryConstraint) new MaxVelocityConstraint(2.0) },
							.05));
		} else {
			RobotContainer.DriveSubsystem.getFollower()
					.follow(new Trajectory(new SimplePathBuilder(robotPose).lineTo(targetPoint).build(),
							new TrajectoryConstraint[] { (TrajectoryConstraint) new MaxAccelerationConstraint(1.25),
									(TrajectoryConstraint) new MaxVelocityConstraint(2.0) },
							.05));
		}

		SmartDashboard.putString("/DriveTrain/TargetPoint", mode.name());
		// RobotContainer.DriveSubsystem.publishPose2d("TargetPoint", targetPoint);
		// RobotContainer.DriveSubsystem.publishPose2d("ModePoint",
		// RobotContainer.DriveSubsystem.points.get(mode));
	}

	// Indicate vision and start the trajectory command

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		LedSubsystem.setAligning(false);

		RobotContainer.DriveSubsystem.getFollower().cancel();
		if (!interrupted) {
			new ParallelDeadlineGroup(new WaitCommand(0.25), new RumbleCommand()).schedule();
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return RobotContainer.DriveSubsystem.getFollower().getCurrentTrajectory().isEmpty();
	}
}