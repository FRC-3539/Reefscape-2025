// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ClimberCommand;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IDConstants;
import frc.robot.constants.IntakeConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private static TalonFX climberMotor;
  // private static CANcoder climberCanCoder;

  public static HashMap<ParentDevice, Alert> connectedClimberAlerts = new HashMap<>();
  public static HashMap<ParentDevice, Alert> wasDisconnectedClimberAlerts = new HashMap<>();

  private void createAlert(ParentDevice device, String deviceName) {
    Alert isAlert = new Alert("Climber Subsystem", deviceName + ": is disconnected", AlertType.kError);
    connectedClimberAlerts.put(device, isAlert);
    Alert wasAlert = new Alert("Climber Subsystem", deviceName + ": was disconnected", AlertType.kError);
    wasDisconnectedClimberAlerts.put(device, wasAlert);
  }

  public ClimberSubsystem() {
    // climberCanCoder = new CANcoder(IDConstants.climberCanCoderID, "rio");

    climberMotor = new TalonFX(IDConstants.climberMotorID, "rio");
    climberMotor.getConfigurator().apply(
        new TalonFXConfiguration().MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive));

    climberMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ClimberConstants.climberSoftMax).withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(ClimberConstants.climberSoftMin));

    climberMotor.getConfigurator()
        .apply(new SlotConfigs().withKP(ClimberConstants.climberkP).withKI(ClimberConstants.climberkI)
            .withKD(ClimberConstants.climberkD).withKV(ClimberConstants.climberkV)
            .withKG(ClimberConstants.climberkG));

    climberMotor.getConfigurator()
        .apply(new MotionMagicConfigs()
            .withMotionMagicAcceleration(ClimberConstants.climberAcceleration)
            .withMotionMagicCruiseVelocity(ClimberConstants.climberCruiseVelocity));

    climberMotor.setNeutralMode(NeutralModeValue.Brake);

    createAlert(climberMotor, "climberMotor");
  }

  public static void setClimberMotor(double voltage) {
    climberMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  // public static double getClimberAngle() {
  // return Units.rotationsToDegrees(
  // climberCanCoder.getAbsolutePosition().getValueAsDouble() -
  // ClimberConstants.climberRestingRotations)
  // + ClimberConstants.restClimberAngle;
  // }
  public static double getClimberPosition() {
    return climberMotor.getPosition().getValueAsDouble();
  }

  // public static void setElevatorPosition(double request) {
  // requestedClimberPos = request;
  // }

  // public static void initializeElevatorPosition() {
  // requestedElevatorPos = getElevatorPosition();
  // }

  public static void setClimberBreakMode(boolean enabled) {
    if (enabled) {
      climberMotor.setNeutralMode(NeutralModeValue.Brake);
    } else {
      climberMotor.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  public static void zeroClimberMotor() {
    climberMotor.setPosition(0);
  }

  public static void enableClimberLimits(boolean enabled) {
    climberMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(enabled)
        .withForwardSoftLimitThreshold(ClimberConstants.climberSoftMax).withReverseSoftLimitEnable(enabled)
        .withReverseSoftLimitThreshold(ClimberConstants.climberSoftMin));
  }

  public void log() {
    SmartDashboard.putNumber("/Climber/ClimberPosition", getClimberPosition());
  }

  @Override
  public void periodic() {
    for (ParentDevice device : connectedClimberAlerts.keySet()) {
      Alert isAlert = connectedClimberAlerts.get(device);
      Alert wasAlert = wasDisconnectedClimberAlerts.get(device);

      if (!device.isConnected()) {
        isAlert.set(true);
        wasAlert.set(false);

      } else if(isAlert.get()) {
        isAlert.set(false);
        wasAlert.set(true);

      }
    }
    log();
    // This method will be called once per scheduler run
  }
}
