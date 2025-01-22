// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  private static TalonFX elevatorMotor;
  public  ElevatorSubsystem() {
    elevatorMotor = new TalonFX(IDConstants.algaeDeployMotorID, "rio");
    elevatorMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
  }

  public static void setElevatorMotor(double voltage) {
    elevatorMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
