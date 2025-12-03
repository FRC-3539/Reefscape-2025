// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.intake.*;
import frc.robot.elevator.*;
import frc.robot.drive.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.misc.LedSubsystem;
import frc.robot.misc.EnumConstants.AlignPoint;
import frc.robot.misc.EnumConstants.ScoringMode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDAutoAlignCommand extends Command {
  /** Creates a new PIDAutoAlignCommand. */

  double translateAccelMax = 2;
  double thetaAccelMax = 2;
  double thetaAccelElevatorUp = 1;
  double translateAccelElevatorUp = 1;

  ProfiledPIDController xController = new ProfiledPIDController(DriveConstants.TranslationkP,
      DriveConstants.TranslationkI, DriveConstants.TranslationkD, new Constraints(4.5, translateAccelMax));

  ProfiledPIDController yController = new ProfiledPIDController(DriveConstants.TranslationkP,
      DriveConstants.TranslationkI, DriveConstants.TranslationkD, new Constraints(4.5, translateAccelMax));

  ProfiledPIDController thetaController = new ProfiledPIDController(DriveConstants.RotationkP,
      DriveConstants.RotationkI, DriveConstants.RotationkD, new Constraints(4.5, thetaAccelMax));


  AlignPoint mode;
  ScoringMode piece;

  Pose2d targetPoint = new Pose2d();

  double maxVelocity = RobotContainer.driveSubsystem.maxVelocity;

  double maxRotationalVelocity = RobotContainer.driveSubsystem.maxRotationalVelocity;

  private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  public PIDAutoAlignCommand(AlignPoint mode, ScoringMode piece) {
    addRequirements(RobotContainer.driveSubsystem);

    this.mode = mode;
    this.piece = piece;

    xController.setTolerance(0.04);
    yController.setTolerance(0.04);
    thetaController.setTolerance(0.035);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // double translateAccel = translateAccelMax;
    // double thetaAccel = thetaAccelMax;

    if(ElevatorSubsystem.requestedElevatorPos>35)
      // translateAccel = translateAccelElevatorUp;
      // thetaAccel = thetaAccelElevatorUp;

    xController.reset(RobotContainer.driveSubsystem.getPose2d().getX(), RobotContainer.driveSubsystem.velocityX);
    yController.reset(RobotContainer.driveSubsystem.getPose2d().getY(), RobotContainer.driveSubsystem.velocityY);
    thetaController.reset(RobotContainer.driveSubsystem.getPose2d().getRotation().getRadians(),
        RobotContainer.driveSubsystem.velocityR);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Pose2d robotPose = RobotContainer.driveSubsystem.getPose2d();

    // Calculate the robot pose after deceleration
    // double deceleratedX = robotPose.getX() + RobotContainer.driveSubsystem.velocityX * translateAccel;
    // double deceleratedY = robotPose.getY() + RobotContainer.driveSubsystem.velocityY * translateAccel;
    // double deceleratedTheta = robotPose.getRotation().getRadians() + RobotContainer.driveSubsystem.velocityR * thetaAccel;

    // robotPose = new Pose2d(deceleratedX, deceleratedY, robotPose.getRotation().plus(new Rotation2d(deceleratedTheta)));

    // Log or use the deceleratedPose as needed

    LedSubsystem.setAligning(true);

    if (mode == AlignPoint.CORALLEFT) {
      targetPoint = robotPose
          .nearest(new ArrayList<Pose2d>(AlignConstants.coralPointsLeft.values()));
    } else if (mode == AlignPoint.CORALRIGHT) {
      targetPoint = robotPose
          .nearest(new ArrayList<Pose2d>(AlignConstants.coralPointsRight.values()));
    } else if (mode == AlignPoint.ALGAE) {
      targetPoint = robotPose
          .nearest(new ArrayList<Pose2d>(AlignConstants.algaePoints.values()));
    } else if (piece == ScoringMode.CLIMB) {
      targetPoint = robotPose
          .nearest(new ArrayList<Pose2d>(AlignConstants.climbPoints.values()));
    } else {
      targetPoint = robotPose
          .nearest(new ArrayList<Pose2d>(AlignConstants.humanPlayerPoints.values()));
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translateAccel = translateAccelMax;
    double thetaAccel = thetaAccelMax;

    if(ElevatorSubsystem.requestedElevatorPos>35)
      translateAccel = translateAccelElevatorUp;
      thetaAccel = thetaAccelElevatorUp;

    SwerveRequest request = new SwerveRequest.Idle();
    var x = xController.calculate(RobotContainer.driveSubsystem.getPose2d().getX(), new State(targetPoint.getX(),0), new Constraints(maxVelocity, translateAccel));
    var y = yController.calculate(RobotContainer.driveSubsystem.getPose2d().getY(), new State(targetPoint.getY(),0), new Constraints(maxVelocity, translateAccel));
    var r = thetaController.calculate(RobotContainer.driveSubsystem.getPose2d().getRotation().getRadians(), new State(targetPoint.getRotation().getRadians(),0), new Constraints(maxVelocity, thetaAccel));
    request = driveFieldCentric
        .withVelocityX(x)
        .withVelocityY(y)
        .withRotationalRate(r);
    RobotContainer.driveSubsystem.applyRequest(request);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LedSubsystem.setAligning(false);
    if (!interrupted) {
      new ParallelDeadlineGroup(new WaitCommand(0.25), new RumbleCommand()).schedule();
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }
}
