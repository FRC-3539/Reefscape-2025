// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.IDConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private static TalonFX climberMotor;
  private static  CANcoder climberCanCoder;

  public ClimberSubsystem() {
    climberCanCoder = new CANcoder(IDConstants.climberCanCoderID, "rio");

    climberMotor = new TalonFX(IDConstants.climberMotorID, "rio");
    climberMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive));

    climberMotor.getConfigurator()
				.apply(new FeedbackConfigs().withFeedbackRemoteSensorID(IDConstants.climberCanCoderID)
						.withRotorToSensorRatio(ClimberConstants.climberMotorToEncoder).withSensorToMechanismRatio(1)
						.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));
  }
  public static void setClimberMotor(double voltage) {
    climberMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }
  public double getClimberAngle() {
		return Units.rotationsToDegrees(
				climberCanCoder.getAbsolutePosition().getValueAsDouble() - ClimberConstants.climberRestingRotations)
				+ ClimberConstants.restClimberAngle;
	}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
