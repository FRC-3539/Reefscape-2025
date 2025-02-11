// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlgaePositionCommand.AlgaeMode;
import frc.robot.commands.CoralPositionCommand.CoralMode;
import frc.robot.commands.SetFunnelPositionCommand.IntakeMode;
import frc.robot.commands.*;
import frc.robot.commands.ScoringCommand.ScoringMode;
import frc.robot.subsystems.*;
//import frc.robot.constants.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed

  public static IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();
  public static ScoringSubsystem ScoringSubsystem = new ScoringSubsystem();
  public static ElevatorSubsystem ElevatorSubsystem = new ElevatorSubsystem();
  public static ClimberSubsystem ClimberSubsystem = new ClimberSubsystem();
  //public static DriveSubsystem DriveSubsystem = TunerConstants.createDrivetrain();
  
	public static CommandXboxController driverController = new CommandXboxController(1);
	public static CommandXboxController operatorController = new CommandXboxController(0);

  public static Trigger rightDriverTrigger = driverController.rightTrigger(0.5);
	public static Trigger rightDriverBumper = driverController.rightBumper();

  public static SendableChooser<Command> chooser = new SendableChooser<Command>();


  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    putAutons();
  
    //Scoring Commands
    NamedCommands.registerCommand("AlgaeScoringCommand", new ScoringCommand(ScoringMode.ALGAE));
    NamedCommands.registerCommand("CoralScoringCommand", new ScoringCommand(ScoringMode.CORAL));

    
    //Setting positions to score Coral
    NamedCommands.registerCommand("TroughCommand", new CoralPositionCommand(CoralMode.TROUGH));
    NamedCommands.registerCommand("CoralLowCommand", new CoralPositionCommand(CoralMode.LOW));
    NamedCommands.registerCommand("CoralMidCommand", new CoralPositionCommand(CoralMode.MID));
    NamedCommands.registerCommand("CoralHighCommand", new CoralPositionCommand(CoralMode.HIGH));

    //Setting positions to score Algae
    NamedCommands.registerCommand("ProcessorCommand", new AlgaePositionCommand(AlgaeMode.PROCESSOR));
    NamedCommands.registerCommand("AlgaeLowCommand", new AlgaePositionCommand(AlgaeMode.REEFLOW));
    NamedCommands.registerCommand("AlgaeHighCommand", new AlgaePositionCommand(AlgaeMode.REEFHIGH));
    NamedCommands.registerCommand("NetCommand", new AlgaePositionCommand(AlgaeMode.NET));

    //Funnel Position Commands
    NamedCommands.registerCommand("HumanPlayerIntakeCommand", new SetFunnelPositionCommand(IntakeMode.HUMAN));

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   * 
   */
  public void putAutons()
  {
    chooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(chooser);
  }
  private void configureBindings() {
    //DriveSubsystem.setDefaultCommand(new DriveCommand());
    
    //Algae Commands
    operatorController.leftTrigger().whileTrue(new AlgaeIntakeCommand());
    operatorController.rightTrigger().whileTrue(new ScoringCommand(ScoringMode.ALGAE));
    operatorController.povDown().onTrue(new AlgaePositionCommand(AlgaeMode.PROCESSOR));
    operatorController.povRight().onTrue(new AlgaePositionCommand(AlgaeMode.REEFLOW));
    operatorController.povUp().onTrue(new AlgaePositionCommand(AlgaeMode.REEFHIGH));
    operatorController.povLeft().onTrue(new AlgaePositionCommand(AlgaeMode.NET));

    //Coral Commands
    operatorController.leftBumper().whileTrue(new SetFunnelPositionCommand(IntakeMode.REVERSE));
    operatorController.axisGreaterThan(0, 0.5).whileTrue(
      new SetFunnelPositionCommand(IntakeMode.HUMAN));
    operatorController.axisLessThan(0, -0.5).whileTrue(
      new SetFunnelPositionCommand(IntakeMode.GROUND));
    operatorController.rightBumper().whileTrue(new ScoringCommand(ScoringMode.CORAL));
    operatorController.a().onTrue(new CoralPositionCommand(CoralMode.TROUGH));
    operatorController.b().onTrue(new CoralPositionCommand(CoralMode.LOW));
    operatorController.y().onTrue(new CoralPositionCommand(CoralMode.MID));
    operatorController.x().onTrue(new CoralPositionCommand(CoralMode.HIGH));
   
    //Climbing Command
    operatorController.rightStick().whileTrue(new ClimberCommand());
    operatorController.start().onTrue(new HomePositionCommand());
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
