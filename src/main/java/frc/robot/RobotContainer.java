// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.annotation.Target;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    NamedCommands.registerCommand("ClimbingLedCommand", new ClimbingLedCommand());
    NamedCommands.registerCommand("CoralDetectedLedCommand", new CoralDetectedLedCommand());
    NamedCommands.registerCommand("IntakingLEDCommand", new IntakingLEDCommand());
    NamedCommands.registerCommand("ElevatorL2Command", new SetElevatorCommand(22.25));
    NamedCommands.registerCommand("ElevatorL4Command", new SetElevatorCommand(74.75));
    NamedCommands.registerCommand("ScoreL2Command", new RotateArmCommand(40, ScoringMode.CORAL));
    NamedCommands.registerCommand("ScoreL4Command", new RotateArmCommand(10, ScoringMode.CORAL));
    NamedCommands.registerCommand("ShootCommand", new ShootCommand());
    NamedCommands.registerCommand("HPFunnelPosition", new RotateFunnelCommand(75));
    NamedCommands.registerCommand("IntakeHPCoral", new IntakeCommand());
    NamedCommands.registerCommand("RotateArmHP", new RotateArmCommand(-130, ScoringMode.CORAL));
    NamedCommands.registerCommand("ScorerIntake", new ReverseShoot());
    NamedCommands.registerCommand("ElevatorAlgeaCommand", new SetElevatorCommand(29));
    NamedCommands.registerCommand("ElevatorHighAlgeaCommand", new SetElevatorCommand(45));

    NamedCommands.registerCommand("AlgeaGrabAngle", new RotateArmCommand(65, ScoringMode.ALGAE));
    NamedCommands.registerCommand("ElevatorAlgeaNetCommand", new SetElevatorCommand(79));
    NamedCommands.registerCommand("AlgeaNetAngle", new RotateArmCommand(90, ScoringMode.ALGAE));
    
    NamedCommands.registerCommand("IntakeAlgae", new ReverseShoot());
    NamedCommands.registerCommand("ShootAlgae", new ShootCommand());
    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(chooser);
  }

  public void putCommands() {
    
  }

  private void configureBindings() {
    DriveSubsystem.setDefaultCommand(new DriveCommand());
    driverController.start().whileTrue(new ZeroGyroCommand());

    // Intake commands
    operatorController.rightTrigger(.2).whileTrue(new ShootCommand());
    operatorController.leftTrigger(0.2).whileTrue(new ReverseShoot());
    // operatorController.b().onTrue(new ElevatorL2Command());
    operatorController.a().onTrue(new ParallelCommandGroup(
      new SetElevatorCommand(ElevatorConstants.troughHeight),
      new RotateArmCommand(ScoringConstants.troughPosition, ScoringMode.CORAL)

    ));
    operatorController.x().onTrue(new ParallelCommandGroup(
      new SetElevatorCommand(ElevatorConstants.coralLowHeight),
      new RotateArmCommand(ScoringConstants.coralLowPosition, ScoringMode.CORAL)
    ));
    operatorController.b().onTrue(new ParallelCommandGroup(
      new SetElevatorCommand(ElevatorConstants.coralMidHeight),
      new RotateArmCommand(ScoringConstants.coralMidPosition, ScoringMode.CORAL)
    ));
    operatorController.y().onTrue(new ParallelCommandGroup(
      new SetElevatorCommand(ElevatorConstants.coralHighHeight),
      new RotateArmCommand(ScoringConstants.coralHighPosition, ScoringMode.CORAL)
    ));
    operatorController.povUp().onTrue(new ParallelCommandGroup(
      new SetElevatorCommand(ElevatorConstants.netHeight),
      new RotateArmCommand(ScoringConstants.netPosition, ScoringMode.ALGAE)
    ));
    operatorController.povDown().onTrue(new ParallelCommandGroup(
      new SetElevatorCommand(ElevatorConstants.groundHeight),
      new RotateArmCommand(ScoringConstants.groundPosition, ScoringMode.ALGAE)
    ));
    operatorController.povRight().onTrue(new ParallelCommandGroup(
      new SetElevatorCommand(ElevatorConstants.algaeHighHeight),
      new RotateArmCommand(ScoringConstants.algaeHighPosition, ScoringMode.ALGAE)
    ));
    operatorController.povLeft().onTrue(new ParallelCommandGroup(
      new SetElevatorCommand(ElevatorConstants.algaeLowHeight),
      new RotateArmCommand(ScoringConstants.algaeLowPosition, ScoringMode.ALGAE)
    ));
    operatorController.axisLessThan(1,-.5).whileTrue(new ParallelCommandGroup(
      new RotateFunnelCommand(IntakeConstants.humanFunnelDeployAngle),
      new SetElevatorCommand(ElevatorConstants.handOffHeight),
      new RotateArmCommand(ScoringConstants.handOffPosition, ScoringMode.CORAL),
      new IntakeCommand(),
      new ReverseShoot()
    ));
    operatorController.axisGreaterThan(1,.5).whileTrue(new ParallelCommandGroup(
      new RotateFunnelCommand(IntakeConstants.groundFunnelDeployAngle),
      new SetElevatorCommand(ElevatorConstants.handOffHeight),
      new RotateArmCommand(ScoringConstants.handOffPosition, ScoringMode.CORAL),
      new IntakeCommand(),
      new ReverseShoot()
    ));
    driverController.x().whileTrue(new BBAutoAlignCommand( AlignPoint.CORALLEFT, null));
    driverController.a().whileTrue(new BBAutoAlignCommand( AlignPoint.ALGAE, null));
    driverController.b().whileTrue(new BBAutoAlignCommand( AlignPoint.CORALRIGHT, null));
  
    // Algae Commands

    // Coral Commands

    // Scoring Commands

    // Other

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
