// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IDConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  private static TalonFX elevatorMotor;
  private static double requestedElevatorPos = 0;

  public  ElevatorSubsystem() {
    elevatorMotor = new TalonFX(IDConstants.algaeDeployMotorID, "rio");
    elevatorMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive));
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    elevatorMotor.getConfigurator().apply(
      new HardwareLimitSwitchConfigs()
        .withReverseLimitEnable(true)
        .withReverseLimitAutosetPositionEnable(true)
        .withReverseLimitAutosetPositionValue(0));
  }

  public static void setElevatorMotor(double voltage) {
    elevatorMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }
  public static double getElevatorPosition() {
		return elevatorMotor.getPosition().getValueAsDouble() * ElevatorConstants.elevatorMotorToInches;
	}
  public static void setElevatorPosition(double request) {
		requestedElevatorPos = request;
	}
  
	public static void initializeElevatorPosition() {
		requestedElevatorPos = getElevatorPosition();
	}
  @Override
  public void periodic() {
    elevatorMotor.setControl(
      new MotionMagicVoltage((requestedElevatorPos / ElevatorConstants.elevatorMotorToInches)));  
    }
}
