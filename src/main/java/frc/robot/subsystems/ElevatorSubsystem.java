// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  private static TalonFX elevatorMotor;
  private static double requestedElevatorPos = 22.5;

  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(16, "rio");
    elevatorMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    elevatorMotor.getConfigurator()
        .apply(new SlotConfigs().withKP(ElevatorConstants.elevatorkP).withKI(ElevatorConstants.elevatorkI)
            .withKD(ElevatorConstants.elevatorkD).withKG(ElevatorConstants.elevatorkG)
            .withGravityType(GravityTypeValue.Elevator_Static));
    elevatorMotor.getConfigurator()
        .apply(new MotionMagicConfigs().withMotionMagicAcceleration(ElevatorConstants.elevatorAcceleration)
            .withMotionMagicCruiseVelocity(ElevatorConstants.elevatorCruiseVelocity));
  }

  public static void setElevatorPosition(double request) {requestedElevatorPos = request;}

  public void log() {
  }

  @Override
  public void periodic() {
   elevatorMotor.setControl(new MotionMagicVoltage((requestedElevatorPos - ElevatorConstants.elevatorHomePositionOffset)/ ElevatorConstants.elevatorInchesPerRotation));
    log();
  }
}
