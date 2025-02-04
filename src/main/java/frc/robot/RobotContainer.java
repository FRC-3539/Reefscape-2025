// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeScoringCommand;
import frc.robot.commands.SetAlgaePositionCommand.AlgaeMode;
import frc.robot.commands.SetCoralPositionCommand.CoralMode;
import frc.robot.commands.SetFunnelPositionCommand.IntakeMode;
import frc.robot.commands.*;
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


  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //DriveSubsystem.setDefaultCommand(new DriveCommand());
    
    operatorController.leftTrigger().whileTrue(new AlgaeIntakeCommand());
    operatorController.rightTrigger().whileTrue(new AlgaeScoringCommand());
		operatorController.rightBumper().whileTrue(new CoralScoringCommand());
    operatorController.leftBumper().whileTrue(new SetFunnelPositionCommand(IntakeMode.REVERSE));
    operatorController.axisGreaterThan(0, 0.5).whileTrue(
      new SetFunnelPositionCommand(IntakeMode.HUMAN));
    operatorController.axisLessThan(0, -0.5).whileTrue(
      new SetFunnelPositionCommand(IntakeMode.GROUND));

    operatorController.a().onTrue(new SetCoralPositionCommand(CoralMode.TROUGH));
    operatorController.b().onTrue(new SetCoralPositionCommand(CoralMode.LOW));
    operatorController.y().onTrue(new SetCoralPositionCommand(CoralMode.MID));
    operatorController.x().onTrue(new SetCoralPositionCommand(CoralMode.HIGH));
    operatorController.povDown().onTrue(new SetAlgaePositionCommand(AlgaeMode.PROCESSOR));
    operatorController.povLeft().onTrue(new SetAlgaePositionCommand(AlgaeMode.REEFLOW));
    operatorController.povUp().onTrue(new SetAlgaePositionCommand(AlgaeMode.REEFHIGH));
    operatorController.povRight().onTrue(new SetAlgaePositionCommand(AlgaeMode.NET));
    operatorController.rightStick().whileTrue(new ClimberCommand());
    operatorController.start().onTrue(new HomePositionCommand());
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
