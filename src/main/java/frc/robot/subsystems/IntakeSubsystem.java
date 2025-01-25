// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IDConstants;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;


public class IntakeSubsystem extends SubsystemBase { 
  
  private static TalonFX algaeDeployMotor,algaeIntakeMotor,coralDeployMotor,coralIntakeMotor;
 
  private static CANcoder coralDeployCanCoder;
  private static double requestedCoralDeployPos = 0;

  public IntakeSubsystem() {

    coralDeployCanCoder = new CANcoder(IDConstants.coralDeployCanCoderID, "rio");

    algaeDeployMotor = new TalonFX(IDConstants.algaeDeployMotorID, "rio");
    algaeDeployMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive));
  
    algaeIntakeMotor = new TalonFX(IDConstants.algaeIntakeMotorID, "rio");
    algaeIntakeMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive));
  
    coralDeployMotor = new TalonFX(IDConstants.coralDeployMotorID, "rio");
    coralDeployMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive));
  
    coralIntakeMotor = new TalonFX(IDConstants.coralIntakeMotorID, "rio");
    coralIntakeMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive));

    coralDeployMotor.getConfigurator()
				.apply(new FeedbackConfigs().withFeedbackRemoteSensorID(IDConstants.coralDeployCanCoderID)
						.withRotorToSensorRatio(IntakeConstants.coralDeployMotorToEncoder).withSensorToMechanismRatio(1)
						.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));
  }
    public static double getCoralDeployAngle() {
		return Units.rotationsToDegrees(
				coralDeployCanCoder.getAbsolutePosition().getValueAsDouble() - IntakeConstants.coralDeployRestingRotations)
				+ IntakeConstants.restCoralDeployAngle;
  }

  public static void setCoralDeployAngle(double angle) {
		requestedCoralDeployPos = angle;
	}

  public static void initializCoralDeployAngle() {
		requestedCoralDeployPos = getCoralDeployAngle();
	}
  
  public static void setAlgaeIntakeMotor(double voltage) {
    algaeIntakeMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public static void setAlgaeDeployMotor(double voltage) {
    algaeDeployMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public static void setCoralDeployMotor(double voltage) {
    coralDeployMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public static void setCoralIntakeMotor(double voltage) {
    coralIntakeMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public static double degreesToCoralDeployRotations(double degrees) {
		return Units.degreesToRotations(degrees - IntakeConstants.restCoralDeployAngle)
				+ IntakeConstants.coralDeployRestingRotations;
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    coralDeployMotor.setControl(new MotionMagicVoltage(degreesToCoralDeployRotations(requestedCoralDeployPos)));
  }
}
