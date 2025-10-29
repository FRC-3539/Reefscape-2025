// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LeftScriptCommand extends Command {
  /** Creates a new LeftScriptCommand. */
  Command script;
  public LeftScriptCommand() {
    script = new PathPlannerAuto("Left Script");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    script.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    script.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return script.isFinished()
      || !MathUtil.isNear(0, RobotContainer.driverController.getLeftTriggerAxis(), 0.1)
      || !MathUtil.isNear(0, RobotContainer.driverController.getRightTriggerAxis(), 0.1);
  }
}
