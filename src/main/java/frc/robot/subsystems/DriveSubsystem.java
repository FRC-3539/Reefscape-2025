// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IDConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.frcteam3539.Byte_Swerve_Lib.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam3539.Byte_Swerve_Lib.control.PidConstants;
import org.frcteam3539.Byte_Swerve_Lib.util.DrivetrainFeedforwardConstants;
import org.frcteam3539.Byte_Swerve_Lib.util.HolonomicFeedforward;

public class DriveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
	/** Creates a new DrivetrainSubsystem. */
	private SwerveRequest swerveRequest = new SwerveRequest.Idle();

	private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

	public double maxVelocity = 0.0;
	public double maxRotationalVelocity = 0.0;

	public Pigeon2 pigeon = new Pigeon2(IDConstants.pigeonID, "rio");

	public enum AlignMode {
		A, B, C, D, E, F, G, H, I, J, K, L, HUMANPLAYER1, HUMANPLAYER2, CLIMB1, CLIMB2, CLIMB3, CLOSEST;
	}

	public static Map<AlignMode, Pose2d> points = new HashMap<>();

	private final HolonomicMotionProfiledTrajectoryFollower follower;

	public DriveSubsystem(SwerveDrivetrainConstants driveTrainConstants,
			SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... modules) {
		super(TalonFX::new, TalonFX::new, CANcoder::new, driveTrainConstants, modules);

		maxVelocity = modules[0].SpeedAt12Volts;

		Translation2d[] moduleLocations = new Translation2d[modules.length];

		for (int i = 0; i < modules.length; i++) {
			moduleLocations[i] = new Translation2d(modules[i].LocationX, modules[i].LocationY);
			this.getModule(i).getDriveMotor().getConfigurator()
					.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(40)
							.withSupplyCurrentLimitEnable(true)
							.withStatorCurrentLimit(modules[i].SlipCurrent).withStatorCurrentLimitEnable(true));
		}

		double dtRadius = new Translation2d().nearest(Arrays.asList(moduleLocations)).getDistance(new Translation2d());
		maxRotationalVelocity = (maxVelocity / dtRadius);

		configureAutoBuilder();

		points.put(AlignMode.A, new Pose2d(3.10, 4.19, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.B, new Pose2d(3.10, 3.86, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.C, new Pose2d(3.65, 2.90, Rotation2d.fromDegrees(60)));
		points.put(AlignMode.D, new Pose2d(3.93, 2.74, Rotation2d.fromDegrees(60)));
		points.put(AlignMode.E, new Pose2d(5.04, 2.74, Rotation2d.fromDegrees(120)));
		points.put(AlignMode.F, new Pose2d(5.33, 2.90, Rotation2d.fromDegrees(120)));
		points.put(AlignMode.G, new Pose2d(5.88, 3.86, Rotation2d.fromDegrees(180)));
		points.put(AlignMode.H, new Pose2d(5.88, 4.19, Rotation2d.fromDegrees(180)));
		points.put(AlignMode.I, new Pose2d(5.33, 5.15, Rotation2d.fromDegrees(-120)));
		points.put(AlignMode.J, new Pose2d(5.04, 5.31, Rotation2d.fromDegrees(-120)));
		points.put(AlignMode.K, new Pose2d(3.93, 5.31, Rotation2d.fromDegrees(-60)));
		points.put(AlignMode.L, new Pose2d(3.65, 5.15, Rotation2d.fromDegrees(-60)));
		points.put(AlignMode.HUMANPLAYER1, new Pose2d(1.75, 1.75, Rotation2d.fromDegrees(50)));
		points.put(AlignMode.HUMANPLAYER2, new Pose2d(1.5, 6.75, Rotation2d.fromDegrees(-50)));
		points.put(AlignMode.CLIMB1, new Pose2d(8.5, 5.25, Rotation2d.fromDegrees(-90)));
		points.put(AlignMode.CLIMB2, new Pose2d(8.5, 5.25, Rotation2d.fromDegrees(-90)));
		points.put(AlignMode.CLIMB3, new Pose2d(8.5, 5.25, Rotation2d.fromDegrees(-90)));

		DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(
				DriveConstants.TranslationkV, DriveConstants.TranslationkA,
				DriveConstants.TranslationkS);

		follower = new HolonomicMotionProfiledTrajectoryFollower(
				new PidConstants(DriveConstants.TranslationkP, DriveConstants.TranslationkI,
						DriveConstants.TranslationkD),
				new PidConstants(DriveConstants.RotationkP, DriveConstants.RotationkI,
						DriveConstants.RotationkD),
				new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

	}

	// public void seedFieldRelative(Trajectory trajectory) {
	// this.seedFieldRelative(trajectory.calculate(0).getPathState().getPose2d());
	// }

	public Pose2d getPose2d() {
		return this.getState().Pose;
	}

	public void applyRequest(SwerveRequest request) {
		this.swerveRequest = request;
	}

	public void log() {
		publishPose2d("/DriveTrain/Pose", getPose2d());
	}

	private void configureAutoBuilder() {
		try {
			var config = RobotConfig.fromGUISettings();
			AutoBuilder.configure(
					() -> getState().Pose, // Supplier of current robot pose
					this::resetPose, // Consumer for seeding pose against auto
					() -> getState().Speeds, // Supplier of current robot speeds
					// Consumer of ChassisSpeeds and feedforwards to drive the robot
					(speeds, feedforwards) -> setControl(
							m_pathApplyRobotSpeeds.withSpeeds(speeds)
									.withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
									.withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
					new PPHolonomicDriveController(
							// PID constants for translation
							new PIDConstants(DriveConstants.TranslationkP, DriveConstants.TranslationkI,
									DriveConstants.TranslationkD),
							// PID constants for rotation
							new PIDConstants(DriveConstants.RotationkP, DriveConstants.RotationkI,
									DriveConstants.RotationkD)),
					config,
					// Assume the path needs to be flipped for Red vs Blue, this is normally the
					// case
					() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
					this // Subsystem for requirements
			);
		} catch (Exception ex) {
			DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
					ex.getStackTrace());
		}
	}

	public static void publishPose2d(String key, Pose2d pose) {
		SmartDashboard.putNumberArray(key, new double[] { pose.getTranslation().getX(), pose.getTranslation().getY(),
				pose.getRotation().getRadians() });
	}

	public Command generateAlignCommand(AlignMode mode) {
		Pose2d targetPoint;
		if (mode == AlignMode.CLOSEST) {
			targetPoint = getPose2d().nearest(new ArrayList<Pose2d>(points.values()));
			for (var entry : points.entrySet()) {
				if (entry.getValue().equals(targetPoint))
					mode = entry.getKey();
			}
		} else {
			targetPoint = points.get(mode);
		}
		if (getPose2d().getTranslation().getDistance(targetPoint.getTranslation()) < 0.01) {
			return Commands.none();
		}

		PathConstraints constraints = new PathConstraints(.5, .5, 1 * Math.PI, 2 * Math.PI); // The constraints for
																								// this path.
		SmartDashboard.putString("/DriveTrain/TargetPoint", mode.name());
		publishPose2d("TargetPoint", targetPoint);
		publishPose2d("ModePoint", points.get(mode));
		Command followPath = AutoBuilder.pathfindToPose(targetPoint, constraints);
		followPath.addRequirements(this);
		return followPath;
	}

	public HolonomicMotionProfiledTrajectoryFollower getFollower() {
		return follower;
	}

	@Override
	public void periodic() {
		log();
		SwerveRequest request = new SwerveRequest.Idle();
		request = swerveRequest;

		var driveSignalOpt = follower.update(getPose2d(), Timer.getFPGATimestamp(), Robot.kDefaultPeriod);

		if (follower.getLastState() != null) {
			VisionSubsystem.publishPose2d("/DriveTrain/PoseRequested",
					follower.getLastState().getPathState().getPose2d());
		} else {
			VisionSubsystem.publishPose2d("/DriveTrain/PoseRequested", new Pose2d());
		}

		if (driveSignalOpt.isPresent()) {
			ChassisSpeeds speeds = driveSignalOpt.get();
			request = new SwerveRequest.RobotCentric().withVelocityX(speeds.vxMetersPerSecond)
					.withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond);
		} else {
			request = swerveRequest;
		}

		this.setControl(request);
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("/DriveTrain/BatteryVoltage", RobotController.getBatteryVoltage());

	}
}
