// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IDConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  private static TalonFX elevatorMotor;
  private static double requestedElevatorPos = 0;
  private static boolean enforcedMinimumHeight, enforcedMaxHandOffHeight = false;

  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(IDConstants.elevatorMotorID, IDConstants.elevatorMotorCanBusName);
    elevatorMotor.getConfigurator().apply(
        new TalonFXConfiguration().MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive));
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);

    elevatorMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(
            (ElevatorConstants.elevatorSoftMax - ElevatorConstants.elevatorHomePositionOffset)
                / ElevatorConstants.elevatorInchesPerRotation)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(
            (ElevatorConstants.elevatorSoftMin - ElevatorConstants.elevatorHomePositionOffset)
                / ElevatorConstants.elevatorInchesPerRotation));

    elevatorMotor.getConfigurator()
        .apply(new SlotConfigs().withKP(ElevatorConstants.elevatorkP).withKI(ElevatorConstants.elevatorkI)
            .withKD(ElevatorConstants.elevatorkD).withKV(ElevatorConstants.elevatorkV)
            .withKG(ElevatorConstants.elevatorkG).withGravityType(GravityTypeValue.Elevator_Static));
    elevatorMotor.getConfigurator()
        .apply(new MotionMagicConfigs()
            .withMotionMagicAcceleration(ElevatorConstants.elevatorAcceleration)
            .withMotionMagicCruiseVelocity(ElevatorConstants.elevatorCruiseVelocity));
  }

  public static void setElevatorMotor(double voltage) {
    // elevatorMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public static double getElevatorPosition() {
    return elevatorMotor.getPosition().getValueAsDouble() *
        ElevatorConstants.elevatorInchesPerRotation + ElevatorConstants.elevatorHomePositionOffset;
  }

  public static void setElevatorPosition(double request) {
    requestedElevatorPos = request;
  }

  public static void initializeElevatorPosition() {
    requestedElevatorPos = getElevatorPosition();
  }

  public static void setElevatorBreakMode(boolean enabled) {
    if (enabled) {
      elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    } else {
      elevatorMotor.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  public static void setEnforcedMinimumHeight(boolean enforce) {
    enforcedMinimumHeight = enforce;
  }

  public static void setEnforcedMaxHandOffHeight(boolean enforce) {
    enforcedMaxHandOffHeight = enforce;
  }

  public static void zeroElevatorMotor() {
    elevatorMotor.setPosition(0);
  }

  public void log() {
    SmartDashboard.putNumber("/Elevator/ElevatorPosition", getElevatorPosition());
    SmartDashboard.putNumber("/Elevator/TargetElevatorPosition", requestedElevatorPos);
  }

  @Override
  public void periodic() {
    double setElevatorPosition = requestedElevatorPos;

    if (enforcedMinimumHeight) {
      setElevatorPosition = Math.max(setElevatorPosition, 21);
    }
    if (enforcedMaxHandOffHeight) {
      setElevatorPosition = Math.min(setElevatorPosition, 35);
    }

    elevatorMotor.setControl(
        new MotionMagicVoltage(((setElevatorPosition - ElevatorConstants.elevatorHomePositionOffset)
            / ElevatorConstants.elevatorInchesPerRotation)));

    log();
  }
}
