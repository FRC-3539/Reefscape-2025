// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.misc.LedSubsystem;
import frc.robot.misc.EnumConstants.*;
import frc.robot.intake.*;

import java.util.ArrayList;
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
			targetPoint = RobotContainer.driveSubsystem.getPose2d()
					.nearest(new ArrayList<Pose2d>(AlignConstants.coralPointsLeft.values()));
		} else if (mode == AlignPoint.CORALRIGHT) {
			targetPoint = RobotContainer.driveSubsystem.getPose2d()
					.nearest(new ArrayList<Pose2d>(AlignConstants.coralPointsRight.values()));
		} else if (mode == AlignPoint.ALGAE) {
			targetPoint = RobotContainer.driveSubsystem.getPose2d()
					.nearest(new ArrayList<Pose2d>(AlignConstants.algaePoints.values()));
			straightAlgae = targetPoint
					.nearest(new ArrayList<Pose2d>(AlignConstants.straightPoints.values()));
		} else if (mode == AlignPoint.LOLLIPOP1 || mode == AlignPoint.LOLLIPOP2 || mode == AlignPoint.LOLLIPOP3) {
			targetPoint = AlignConstants.getLollipopAlignPoint(mode, RobotContainer.driveSubsystem.getPose2d());
		} else if (mode == AlignPoint.BARGE) {
			targetPoint = AlignConstants.getBargeAlignPoint(RobotContainer.driveSubsystem.getPose2d());
		} else if (piece == ScoringMode.CLIMB) {
			targetPoint = RobotContainer.driveSubsystem.getPose2d()
					.nearest(new ArrayList<Pose2d>(AlignConstants.climbPoints.values()));
		} else if (piece == ScoringMode.CORAL) {
			if (AlignConstants.coralPointsLeft.containsKey(mode)) {
				targetPoint = AlignConstants.coralPointsLeft.get(mode);
			} else {
				targetPoint = AlignConstants.coralPointsRight.get(mode);
			}
		} else {

			targetPoint = AlignConstants.humanPlayerPoints.get(mode);

			// targetPoint = RobotContainer.DriveSubsystem.getPose2d()
			// .nearest(new ArrayList<Pose2d>(AlignConstants.humanPlayerPoints.values()));
		}

		Pose2d robotPose = RobotContainer.driveSubsystem.getPose2d();

		// Generate trajectory command to nearest coordinate
		if(mode == AlignPoint.BARGE)
		{
			RobotContainer.driveSubsystem.getFollower()
					.follow(new Trajectory(new SimplePathBuilder(robotPose).lineTo(targetPoint).build(),
							new TrajectoryConstraint[] { (TrajectoryConstraint) new MaxAccelerationConstraint(1),
									(TrajectoryConstraint) new MaxVelocityConstraint(2) },
							.05));
		}
		if (mode == AlignPoint.ALGAE) {
			RobotContainer.driveSubsystem.getFollower()
					.follow(new Trajectory(
							new SimplePathBuilder(robotPose).lineTo(targetPoint).lineTo(straightAlgae).build(),
							new TrajectoryConstraint[] { (TrajectoryConstraint) new MaxAccelerationConstraint(1.25),
									(TrajectoryConstraint) new MaxVelocityConstraint(2.0) },
							.05));
		} else {
			RobotContainer.driveSubsystem.getFollower()
					.follow(new Trajectory(new SimplePathBuilder(robotPose).lineTo(targetPoint).build(),
							new TrajectoryConstraint[] { (TrajectoryConstraint) new MaxAccelerationConstraint(1.75),
									(TrajectoryConstraint) new MaxVelocityConstraint(3.5) },
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

		RobotContainer.driveSubsystem.getFollower().cancel();
		if (!interrupted) {
			new ParallelDeadlineGroup(new WaitCommand(0.25), new RumbleCommand()).schedule();
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return RobotContainer.driveSubsystem.getFollower().getCurrentTrajectory().isEmpty();
	}
}