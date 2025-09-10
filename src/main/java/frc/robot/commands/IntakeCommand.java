// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */

  Timer timer;
  public IntakeCommand() {
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakeSubsystem.setouterfunnelmotor(3.5);
    IntakeSubsystem.setinnerfunnelmotor(0);
    timer.restart();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.hasElapsed(0.6)){
      timer.restart();
    }
    if(timer.hasElapsed(0.5)){
      
    IntakeSubsystem.setinnerfunnelmotor(0);
    return;
    

    }
    if (MathUtil.isNear(ScoringSubsystem.getRotateAngle(),ScoringSubsystem.getRequestedArmPosition(),3))
    {
      IntakeSubsystem.setinnerfunnelmotor(8);

    } else
      IntakeSubsystem.setinnerfunnelmotor(0);

  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.setouterfunnelmotor(0);
    IntakeSubsystem.setinnerfunnelmotor(0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
