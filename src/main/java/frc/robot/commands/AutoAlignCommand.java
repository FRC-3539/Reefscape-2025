// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.AlignConstants.AlignMode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignCommand extends Command {

  AlignMode mode;
  Command alignCommand;

  /** Creates a new AutoAlignCommand. */
  public AutoAlignCommand(AlignMode targetPoint) {
    mode = targetPoint;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not
    // use holonomic rotation.
    PathPlannerPath.clearCache();

    alignCommand = RobotContainer.DriveSubsystem.generateAlignCommand(mode);
    alignCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    alignCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return alignCommand.isFinished();
  }
}
