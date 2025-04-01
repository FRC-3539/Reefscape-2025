// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AlignConstants;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IDConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ScoringConstants;
import frc.robot.Generated.TunerConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.constants.EnumConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static AlignConstants alignConstants = new AlignConstants();
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
  public static LedSubsystem LEDSubsystem = new LedSubsystem(true);

  public static CommandXboxController driverController = new CommandXboxController(1);
  public static CommandXboxController operatorController = new CommandXboxController(0);
  public static CommandGenericHID buttonBox = new CommandGenericHID(3);

  public static Trigger rightDriverTrigger = driverController.rightTrigger(0.5);
  public static Trigger rightDriverBumper = driverController.rightBumper();
  public static BooleanSupplier coralDetected = () -> ScoringSubsystem.coralDetected()
      && MathUtil.isNear(ScoringSubsystem.getRotateAngle(), ScoringConstants.handOffPosition, 10);
  public static Trigger coralTrigger = new Trigger(coralDetected);

  public static SendableChooser<Command> chooser = new SendableChooser<Command>();

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    putAutons();
    putCommands();
    VisionSubsystem.start();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   * 
   */
  public void putAutons() {

    // Scoring Commands
    NamedCommands.registerCommand("ScoringCommand", new ScoringCommand(false));

    // Setting positions to score Coral
    NamedCommands.registerCommand("TroughCommand", new CoralPositionCommand(CoralMode.TROUGH, true));
    NamedCommands.registerCommand("CoralLowCommand", new CoralPositionCommand(CoralMode.LOW, true));
    NamedCommands.registerCommand("CoralMidCommand", new CoralPositionCommand(CoralMode.MID, true));
    NamedCommands.registerCommand("CoralHighCommand", new CoralPositionCommand(CoralMode.HIGH, true));

    // Setting positions to score Algae
    NamedCommands.registerCommand("ProcessorCommand", new AlgaePositionCommand(AlgaeMode.PROCESSOR, true));
    NamedCommands.registerCommand("AlgaeLowCommand", new AlgaePositionCommand(AlgaeMode.REEFLOW, true));
    NamedCommands.registerCommand("AlgaeHighCommand", new AlgaePositionCommand(AlgaeMode.REEFHIGH, true));
    NamedCommands.registerCommand("NetCommand", new AlgaePositionCommand(AlgaeMode.NET, true));

    // Funnel Position Commands
    NamedCommands.registerCommand("HumanPlayerIntakeCommand", new HandOffCommand(true, IntakeMode.HUMAN));
    NamedCommands.registerCommand("FunnelRangeCommand", new FunnelRangeCommand());

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
    coralTrigger.onTrue(new ParallelDeadlineGroup(new WaitCommand(0.5), new RumbleCommand()));

    DriveSubsystem.setDefaultCommand(new DriveCommand());
    ClimberSubsystem.setDefaultCommand(new ClimberCommand());

    driverController.start().whileTrue(new ZeroGyroCommand());
    driverController.a().whileTrue(new PIDAutoAlignCommand(AlignPoint.ALGAE, ScoringMode.ALGAE));
    driverController.b().whileTrue(new PIDAutoAlignCommand(AlignPoint.CORALRIGHT, ScoringMode.CORAL));
    driverController.x().whileTrue(new PIDAutoAlignCommand(AlignPoint.CORALLEFT, ScoringMode.CLIMB));
    driverController.y().whileTrue(new PIDAutoAlignCommand(AlignPoint.HUMANPLAYER2, ScoringMode.HUMANPLAYER));
    // driverController.a().whileTrue(new BBAutoAlignCommand(AlignPoint.ALGAE,
    // ScoringMode.ALGAE));
    // driverController.b().whileTrue(new BBAutoAlignCommand(AlignPoint.CORALRIGHT,
    // ScoringMode.CORAL));
    // driverController.x().whileTrue(new BBAutoAlignCommand(AlignPoint.CORALLEFT,
    // ScoringMode.CORAL));
    // driverController.y().whileTrue(new
    // BBAutoAlignCommand(AlignPoint.HUMANPLAYER2, ScoringMode.HUMANPLAYER));

    // Intake commands\
    operatorController.axisGreaterThan(1, 0.5).whileTrue(
        new HandOffCommand(false, IntakeMode.GROUND));
    operatorController.axisLessThan(1, -0.5).whileTrue(
        new HandOffCommand(false, IntakeMode.HUMAN));
    
    operatorController.rightBumper().whileTrue(new SetFunnelPositionCommand(IntakeMode.HUMAN));
    operatorController.leftBumper().whileTrue(new SetFunnelPositionCommand(IntakeMode.GROUND));

    // Algae Commands
    operatorController.povDown().onTrue(new AlgaePositionCommand(AlgaeMode.GROUND, false));
    operatorController.povLeft().onTrue(new AlgaePositionCommand(AlgaeMode.REEFLOW, false));
    operatorController.povRight().onTrue(new AlgaePositionCommand(AlgaeMode.REEFHIGH, false));
    operatorController.povUp().onTrue(new AlgaePositionCommand(AlgaeMode.NET, false));

    // Coral Commands
    operatorController.back().whileTrue(new SetFunnelPositionCommand(IntakeMode.REVERSE));
    operatorController.b().onTrue(new CoralPositionCommand(CoralMode.TROUGH, false));
    operatorController.a().onTrue(new CoralPositionCommand(CoralMode.LOW, false));
    operatorController.x().onTrue(new CoralPositionCommand(CoralMode.MID, false));
    operatorController.y().onTrue(new CoralPositionCommand(CoralMode.HIGH, false));

    // Scoring Commands
    operatorController.leftTrigger().whileTrue(new ScoringCommand(true));
    operatorController.rightTrigger().whileTrue(new ScoringCommand(false));

    // Climbing Command
    // operatorController.start().onTrue(new HomePositionCommand());

    // Other
    operatorController.start().onTrue(new ClimbPositionCommand());

    buttonBox.button(14).whileTrue(new BBAutoAlignCommand(AlignPoint.A, ScoringMode.CORAL));
    buttonBox.button(15).whileTrue(new BBAutoAlignCommand(AlignPoint.B, ScoringMode.CORAL));
    buttonBox.button(16).whileTrue(new BBAutoAlignCommand(AlignPoint.C, ScoringMode.CORAL));
    buttonBox.button(12).whileTrue(new BBAutoAlignCommand(AlignPoint.D, ScoringMode.CORAL));
    buttonBox.button(8).whileTrue(new BBAutoAlignCommand(AlignPoint.E, ScoringMode.CORAL));
    buttonBox.button(4).whileTrue(new BBAutoAlignCommand(AlignPoint.F, ScoringMode.CORAL));
    buttonBox.button(3).whileTrue(new BBAutoAlignCommand(AlignPoint.G, ScoringMode.CORAL));
    buttonBox.button(2).whileTrue(new BBAutoAlignCommand(AlignPoint.H, ScoringMode.CORAL));
    buttonBox.button(1).whileTrue(new BBAutoAlignCommand(AlignPoint.I, ScoringMode.CORAL));
    buttonBox.button(5).whileTrue(new BBAutoAlignCommand(AlignPoint.J, ScoringMode.CORAL));
    buttonBox.button(9).whileTrue(new BBAutoAlignCommand(AlignPoint.K, ScoringMode.CORAL));
    buttonBox.button(13).whileTrue(new BBAutoAlignCommand(AlignPoint.L, ScoringMode.CORAL));
    buttonBox.button(10).whileTrue(new BBAutoAlignCommand(AlignPoint.HUMANPLAYER1, ScoringMode.HUMANPLAYER));
    buttonBox.button(11).whileTrue(new BBAutoAlignCommand(AlignPoint.HUMANPLAYER2, ScoringMode.HUMANPLAYER));

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
