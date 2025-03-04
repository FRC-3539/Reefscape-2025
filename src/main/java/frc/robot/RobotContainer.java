// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlgaePositionCommand.AlgaeMode;
import frc.robot.commands.CoralPositionCommand.CoralMode;
import frc.robot.commands.SetFunnelPositionCommand.IntakeMode;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IDConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ScoringConstants;
import frc.robot.Generated.TunerConstants;
import frc.robot.commands.*;
import frc.robot.commands.ScoringCommand.ScoringMode;
import frc.robot.subsystems.*;
//import frc.robot.constants.*;
import frc.robot.subsystems.DriveSubsystem.AlignMode;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static ClimberConstants climberConstants = new ClimberConstants();
  public static DriveConstants driveConstants = new DriveConstants();
  public static ElevatorConstants elevatorConstants = new ElevatorConstants();
  public static IDConstants idConstants = new IDConstants();
  public static IntakeConstants intakeConstants = new IntakeConstants();
  public static ScoringConstants scoringConstants = new ScoringConstants();

  public static IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();
  public static ScoringSubsystem ScoringSubsystem = new ScoringSubsystem();
  public static ElevatorSubsystem ElevatorSubsystem = new ElevatorSubsystem();
  public static ClimberSubsystem ClimberSubsystem = new ClimberSubsystem();
  public static DriveSubsystem DriveSubsystem = TunerConstants.createDrivetrain();
  public static VisionSubsystem VisionSubsystem = new VisionSubsystem();
  
	public static CommandXboxController driverController = new CommandXboxController(1);
	public static CommandXboxController operatorController = new CommandXboxController(0);

  public static Trigger rightDriverTrigger = driverController.rightTrigger(0.5);
	public static Trigger rightDriverBumper = driverController.rightBumper();

  public static SendableChooser<Command> chooser = new SendableChooser<Command>();


  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    putAutons();
    putCommands();
    VisionSubsystem.start();

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
      
    //Scoring Commands
    NamedCommands.registerCommand("AlgaeScoringCommand", new ScoringCommand(ScoringMode.ALGAE));
    NamedCommands.registerCommand("CoralScoringCommand", new ScoringCommand(ScoringMode.CORAL));

    
    //Setting positions to score Coral
    NamedCommands.registerCommand("TroughCommand", new CoralPositionCommand(CoralMode.TROUGH, true));
    NamedCommands.registerCommand("CoralLowCommand", new CoralPositionCommand(CoralMode.LOW, true));
    NamedCommands.registerCommand("CoralMidCommand", new CoralPositionCommand(CoralMode.MID, true));
    NamedCommands.registerCommand("CoralHighCommand", new CoralPositionCommand(CoralMode.HIGH, true));


    //Setting positions to score Algae
    NamedCommands.registerCommand("ProcessorCommand", new AlgaePositionCommand(AlgaeMode.PROCESSOR, true));
    NamedCommands.registerCommand("AlgaeLowCommand", new AlgaePositionCommand(AlgaeMode.REEFLOW, true));
    NamedCommands.registerCommand("AlgaeHighCommand", new AlgaePositionCommand(AlgaeMode.REEFHIGH, true));
    NamedCommands.registerCommand("NetCommand", new AlgaePositionCommand(AlgaeMode.NET, true));

    //Funnel Position Commands
    NamedCommands.registerCommand("HumanPlayerIntakeCommand", new SetFunnelPositionCommand(IntakeMode.HUMAN));

    chooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(chooser);
  }
  public void putCommands() {
		SmartDashboard.putData(new DisableElevatorBreakModeCommand().ignoringDisable(true));
    SmartDashboard.putData(new DisableFunnelBreakModeCommand().ignoringDisable(true));
    SmartDashboard.putData(new DisableClimberBreakModeCommand().ignoringDisable(true));
    SmartDashboard.putData(new DisableScoringBreakModeCommand().ignoringDisable(true));
    SmartDashboard.putData(new ZeroElevatorCommand().ignoringDisable(true));
    SmartDashboard.putData(new ZeroClimberCommand());
	}

  private void configureBindings() {
    DriveSubsystem.setDefaultCommand(new DriveCommand());
    ClimberSubsystem.setDefaultCommand(new ClimberCommand());

    driverController.start().whileTrue(new ZeroGyroCommand());
    driverController.a().whileTrue(new BBAutoAlignCommand(AlignMode.A));
    driverController.b().whileTrue(new BBAutoAlignCommand(AlignMode.CLOSEST));


    // Intake commands
    operatorController.axisGreaterThan(1, 0.5).whileTrue(
      new HandOffCommand(false, IntakeMode.GROUND));
    operatorController.axisLessThan(1, -0.5).whileTrue(
      new HandOffCommand(false, IntakeMode.HUMAN));

    //Algae Commands
    operatorController.povDown().onTrue(new AlgaePositionCommand(AlgaeMode.GROUND, false));
    operatorController.povLeft().onTrue(new AlgaePositionCommand(AlgaeMode.REEFLOW, false));
    operatorController.povRight().onTrue(new AlgaePositionCommand(AlgaeMode.REEFHIGH, false));
    operatorController.povUp().onTrue(new AlgaePositionCommand(AlgaeMode.NET, false));

    //Coral Commands
    // operatorController.leftBumper().whileTrue(new SetFunnelPositionCommand(IntakeMode.REVERSE));
    operatorController.a().onTrue(new CoralPositionCommand(CoralMode.TROUGH, false));
    operatorController.x().onTrue(new CoralPositionCommand(CoralMode.LOW, false));
    operatorController.b().onTrue(new CoralPositionCommand(CoralMode.MID, false));
    operatorController.y().onTrue(new CoralPositionCommand(CoralMode.HIGH, false));
   
    // Scoring Commands
    operatorController.leftTrigger().whileTrue(new ScoringCommand(ScoringMode.ALGAE));
    operatorController.rightTrigger().whileTrue(new ScoringCommand(ScoringMode.CORAL));

    //Climbing Command
    // operatorController.start().onTrue(new HomePositionCommand());

    // Other
    operatorController.start().onTrue(new HandOffCommand(false, IntakeMode.HANDOFF));
    
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
