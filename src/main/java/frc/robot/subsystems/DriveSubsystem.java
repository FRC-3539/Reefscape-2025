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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IDConstants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class DriveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
	/** Creates a new DrivetrainSubsystem. */
	private SwerveRequest swerveRequest = new SwerveRequest.Idle();

	private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

	public double maxVelocity = 0.0;
	public double maxRotationalVelocity = 0.0;

	public Pigeon2 pigeon = new Pigeon2(IDConstants.pigeonID, "rio");

	public enum AlignMode {
		A, B, C, D, E, F, G, H, I, J, K, L, HUMANPLAYER1, HUMANPLAYER2, CLIMB1, CLIMB2, CLIMB3;
	}

	public static Map<AlignMode, Pose2d> points = new HashMap<>();

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

		points.put(AlignMode.A, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.B, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.C, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.D, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.E, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.F, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.G, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.H, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.I, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.J, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.K, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.L, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.HUMANPLAYER1, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.HUMANPLAYER2, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.CLIMB1, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.CLIMB2, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
		points.put(AlignMode.CLIMB3, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));

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
		List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
				getPose2d(),
				points.get(mode));

		PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for
																								// this path.
		// PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); //
		// You can also use unlimited constraints, only limited by motor torque and
		// nominal battery voltage

		// Create the path using the waypoints created above
		PathPlannerPath path = new PathPlannerPath(
				waypoints,
				constraints,
				null, // The ideal starting state, this is only relevant for pre-planned paths, so can
						// be null for on-the-fly paths.
				new GoalEndState(0.0, points.get(mode).getRotation()) // Goal end state. You can set a holonomic
																		// rotation here. If using a differential
																		// drivetrain, the rotation will have no effect.
		);

		// Prevent the path from being flipped if the coordinates are already correct
		path.preventFlipping = true;

		return AutoBuilder.followPath(path);

	}

	@Override
	public void periodic() {
		SwerveRequest request = new SwerveRequest.Idle();
		request = swerveRequest;

		this.setControl(request);
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("/DriveTrain/BatteryVoltage", RobotController.getBatteryVoltage());

	}
}
