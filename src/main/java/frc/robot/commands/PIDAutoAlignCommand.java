// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.AlignConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.EnumConstants.AlignPoint;
import frc.robot.constants.EnumConstants.ScoringMode;
import frc.robot.subsystems.LedSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDAutoAlignCommand extends Command {
  /** Creates a new PIDAutoAlignCommand. */
  ProfiledPIDController xController = new ProfiledPIDController(DriveConstants.TranslationkP,
      DriveConstants.TranslationkI, DriveConstants.TranslationkD, new Constraints(1, 1));

  ProfiledPIDController yController = new ProfiledPIDController(DriveConstants.TranslationkP,
      DriveConstants.TranslationkI, DriveConstants.TranslationkD, new Constraints(1, 1));

  ProfiledPIDController thetaController = new ProfiledPIDController(DriveConstants.RotationkP,
      DriveConstants.RotationkI, DriveConstants.RotationkD, new Constraints(1, 1));

  AlignPoint mode;
  ScoringMode piece;

  public PIDAutoAlignCommand(AlignPoint mode, ScoringMode piece) {
    addRequirements(RobotContainer.DriveSubsystem);

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
    LedSubsystem.setAligning(true);

    Pose2d targetPoint;
    if (mode == AlignPoint.CORALLEFT) {
      if (piece == ScoringMode.CORAL) {
        targetPoint = RobotContainer.DriveSubsystem.getPose2d()
            .nearest(new ArrayList<Pose2d>(AlignConstants.coralPointsLeft.values()));
      } else if (piece == ScoringMode.ALGAE) {
        targetPoint = RobotContainer.DriveSubsystem.getPose2d()
            .nearest(new ArrayList<Pose2d>(AlignConstants.algaePoints.values()));
      } else if (piece == ScoringMode.CLIMB) {
        targetPoint = RobotContainer.DriveSubsystem.getPose2d()
            .nearest(new ArrayList<Pose2d>(AlignConstants.climbPoints.values()));
      } else {
        targetPoint = RobotContainer.DriveSubsystem.getPose2d()
            .nearest(new ArrayList<Pose2d>(AlignConstants.humanPlayerPoints.values()));
      }
    } else {
      targetPoint = AlignConstants.coralPointsLeft.get(mode);
    }
    xController.setGoal(new State(targetPoint.getX(), 0));
    yController.setGoal(new State(targetPoint.getY(), 0));
    thetaController.setGoal(new State(targetPoint.getRotation().getRadians(), 0));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xController.calculate(RobotContainer.DriveSubsystem.getPose2d().getX());
    yController.calculate(RobotContainer.DriveSubsystem.getPose2d().getY());
    thetaController.calculate(RobotContainer.DriveSubsystem.getPose2d().getRotation().getRadians());
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
