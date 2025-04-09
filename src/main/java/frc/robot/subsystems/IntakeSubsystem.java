// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IDConstants;

import java.text.DecimalFormat;
import java.util.HashMap;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class IntakeSubsystem extends SubsystemBase {

  private static TalonFX funnelDeployMotor; // , algaeDeployMotor,algaeIntakeMotor;
  private static TalonFXS funnelIntakeMotor, coralIntakeMotor;

  private static CANcoder funnelDeployCanCoder;
  private static CANrange funnelRange, humanPlayerRange;
  private static double requestedFunnelDeployPos = 0;
  DecimalFormat df = new DecimalFormat("#.00000");
  LinearFilter filter = LinearFilter.movingAverage(5);

  public static HashMap<ParentDevice, Alert> connectedIntakeAlerts = new HashMap<>();
  public static HashMap<ParentDevice, Alert> wasDisconnectedIntakeAlerts = new HashMap<>();

  private void createAlert(ParentDevice device, String deviceName) {
    Alert isAlert = new Alert("Intake Subsystem", deviceName + ": is disconnected", AlertType.kError);
    connectedIntakeAlerts.put(device, isAlert);
    Alert wasAlert = new Alert("Intake Subsystem", deviceName + ": was disconnected", AlertType.kError);
    wasDisconnectedIntakeAlerts.put(device, wasAlert);
  }

  public IntakeSubsystem() {

    funnelDeployCanCoder = new CANcoder(IDConstants.funnelDeployCanCoderID, "rio");

    funnelDeployCanCoder.getConfigurator()
        .apply(new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(IntakeConstants.funnelDeployDiscontPoint)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withMagnetOffset(IntakeConstants.funnelDeployOffset));

    // algaeDeployMotor = new TalonFX(IDConstants.algaeDeployMotorID, "Default
    // Name");
    // algaeDeployMotor.getConfigurator().apply(
    // new TalonFXConfiguration().MotorOutput
    // .withInverted(InvertedValue.Clockwise_Positive));

    // algaeIntakeMotor = new TalonFX(IDConstants.algaeIntakeMotorID, "rio");
    // algaeIntakeMotor.getConfigurator().apply(
    // new TalonFXConfiguration().MotorOutput
    // .withInverted(InvertedValue.CounterClockwise_Positive));

    funnelDeployMotor = new TalonFX(IDConstants.funnelDeployMotorID, "rio");
    if (IntakeConstants.isInverted == 0) {
      funnelDeployMotor.getConfigurator().apply(
          new TalonFXConfiguration().MotorOutput
              .withInverted(InvertedValue.Clockwise_Positive));
    } else {
      funnelDeployMotor.getConfigurator().apply(
          new TalonFXConfiguration().MotorOutput
              .withInverted(InvertedValue.CounterClockwise_Positive));
    }

    funnelDeployMotor.setNeutralMode(NeutralModeValue.Brake);

    coralIntakeMotor = new TalonFXS(IDConstants.coralIntakeMotorID, "rio");
    coralIntakeMotor.getConfigurator().apply(new TalonFXSConfiguration()
        .withCommutation(new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST)));
    coralIntakeMotor.getConfigurator().apply(
        new TalonFXSConfiguration().MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive));

    funnelIntakeMotor = new TalonFXS(IDConstants.funnelIntakeMotorID, "rio");
    funnelIntakeMotor.getConfigurator().apply(new TalonFXSConfiguration()
        .withCommutation(new CommutationConfigs()
            .withMotorArrangement(MotorArrangementValue.Minion_JST)));
    funnelIntakeMotor.getConfigurator().apply(
        new TalonFXSConfiguration().MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive));

    // algaeDeployMotor.getConfigurator().apply(new
    // SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
    // .withForwardSoftLimitThreshold(IntakeConstants.algaeDeploySoftMax).withReverseSoftLimitEnable(true)
    // .withReverseSoftLimitThreshold(IntakeConstants.algaeDeploySoftMin));

    // algaeDeployMotor.getConfigurator()
    // .apply(new
    // SlotConfigs().withKP(IntakeConstants.algaeDeploykP).withKI(IntakeConstants.algaeDeploykI)
    // .withKD(IntakeConstants.algaeDeploykD).withKV(IntakeConstants.algaeDeploykV)
    // .withKG(IntakeConstants.algaeDeploykG));

    // algaeDeployMotor.getConfigurator()
    // .apply(new MotionMagicConfigs()
    // .withMotionMagicAcceleration(IntakeConstants.algaeDeployAcceleration)
    // .withMotionMagicCruiseVelocity(IntakeConstants.algaeDeployCruiseVelocity));

    funnelDeployMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(degreesToFunnelDeployRotations(IntakeConstants.funnelDeploySoftMax))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(degreesToFunnelDeployRotations(IntakeConstants.funnelDeploySoftMin)));

    funnelDeployMotor.getConfigurator()
        .apply(new FeedbackConfigs().withFeedbackRemoteSensorID(IDConstants.funnelDeployCanCoderID)
            .withRotorToSensorRatio(IntakeConstants.funnelDeployMotorToMechanism).withSensorToMechanismRatio(1)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));

    funnelDeployMotor.getConfigurator()
        .apply(new SlotConfigs().withKP(IntakeConstants.funnelDeploykP).withKI(IntakeConstants.funnelDeploykI)
            .withKD(IntakeConstants.funnelDeploykD).withKV(IntakeConstants.funnelDeploykV)
            .withKG(IntakeConstants.funnelDeploykG).withGravityType(GravityTypeValue.Arm_Cosine));

    funnelDeployMotor.getConfigurator()
        .apply(new MotionMagicConfigs()
            .withMotionMagicAcceleration(IntakeConstants.funnelDeployAcceleration)
            .withMotionMagicCruiseVelocity(IntakeConstants.funnelDeployCruiseVelocity));
    funnelRange = new CANrange(IDConstants.funnelRangeID, "rio");

    funnelRange.getConfigurator().apply(new FovParamsConfigs()
        .withFOVCenterX(0)
        .withFOVCenterY(0)
        .withFOVRangeX(6.75)
        .withFOVRangeY(6.75));

    humanPlayerRange = new CANrange(IDConstants.humanPlayerRangeID, "rio");

    humanPlayerRange.getConfigurator().apply(new FovParamsConfigs()
        .withFOVCenterX(0)
        .withFOVCenterY(0)
        .withFOVRangeX(6.75)
        .withFOVRangeY(6.75));

    createAlert(funnelDeployCanCoder, "funnelDeployCanConder");
    createAlert(funnelDeployMotor, "funnelDeployMotor");
    createAlert(coralIntakeMotor, "coralIntakeMotor");
    createAlert(funnelIntakeMotor, "funnelIntakeMotor");
    createAlert(funnelRange, "funnelRange");
    createAlert(humanPlayerRange, "humanPlayerRange");
  }

  public static double getFunnelDeployAngle() {
    return Units.rotationsToDegrees(
        funnelDeployCanCoder.getAbsolutePosition().getValueAsDouble());
  }

  public static void setFunnelDeployAngle(double angle) {
    requestedFunnelDeployPos = angle;
  }

  public static void initializFunnelDeployAngle() {
    requestedFunnelDeployPos = getFunnelDeployAngle();
  }

  // public static void setAlgaeIntakeMotor(double voltage) {
  // algaeIntakeMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  // }

  // public static void setAlgaeDeployMotor(double voltage) {
  // algaeDeployMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  // }
  // public static double getAlgaeDeployPosition() {
  // return algaeDeployMotor.getPosition().getValueAsDouble();
  // }

  public static void setCoralIntakeMotor(double voltage) {
    coralIntakeMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public static void setFunnelIntakeMotor(double voltage) {
    funnelIntakeMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public static double degreesToFunnelDeployRotations(double degrees) {
    return Units.degreesToRotations(degrees);
  }

  public static double getFunnelDistance() {
    return funnelRange.getDistance().getValueAsDouble();
  }

  public static double getHumanPlayerDistance() {
    return humanPlayerRange.getDistance().getValueAsDouble();
  }

  public static void setFunnelBreakMode(boolean enabled) {
    if (enabled) {
      funnelDeployMotor.setNeutralMode(NeutralModeValue.Brake);
    } else {
      funnelDeployMotor.setNeutralMode(NeutralModeValue.Coast);
    }

  }

  public static double getAutonFunnelAngle() {
    return 90 + (0.5 - IntakeSubsystem.getHumanPlayerDistance()) * 100;
  }

  public void log() {
    SmartDashboard.putNumber("/Intake/FunnelAngle", getFunnelDeployAngle());
    SmartDashboard.putNumber("/Intake/TargetFunnelAngle", requestedFunnelDeployPos);
    SmartDashboard.putNumber("/Intake/FunnelDistance", getFunnelDistance());
    SmartDashboard.putNumber("/Intake/HumanPlayerDistance", getHumanPlayerDistance());
    SmartDashboard.putNumber("/Intake/AutonFunnelAngle", filter.lastValue());

    // SmartDashboard.putNumber("/Intake/AlgaeDeployPosition",
    // getAlgaeDeployPosition());

  }

  @Override
  public void periodic() {
    for (ParentDevice device : connectedIntakeAlerts.keySet()) {
      Alert isAlert = connectedIntakeAlerts.get(device);
      Alert wasAlert = wasDisconnectedIntakeAlerts.get(device);

      if (!device.isConnected()) {
        isAlert.set(true);
        wasAlert.set(false);

      } else if (isAlert.get()) {
        isAlert.set(false);
        wasAlert.set(true);

      }
    }
    double value = filter.calculate(getAutonFunnelAngle());
    log();
    // This method will be called once per scheduler run
    funnelDeployMotor.setControl(new MotionMagicVoltage(degreesToFunnelDeployRotations(requestedFunnelDeployPos)));
  }
}
