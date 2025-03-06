// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ScoringConstants;
import frc.robot.constants.IDConstants;

public class ScoringSubsystem extends SubsystemBase {
  private static TalonFX rotateMotor, scoringMotor;
  private static CANcoder rotateCanCoder;
  private static CANrange algaeRange, coralRange;
  private static double requestedRotatePos = 0;
  DecimalFormat df = new DecimalFormat("#.00000");
  private static double scoringRestrictedMin = 35;

  public ScoringSubsystem() {
    scoringMotor = new TalonFX(IDConstants.scoringMotorID, "rio");
    scoringMotor.getConfigurator().apply(
        new TalonFXConfiguration().MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive));
    scoringMotor.setNeutralMode(NeutralModeValue.Brake);

    rotateMotor = new TalonFX(IDConstants.rotateMotorID, "rio");
    rotateMotor.getConfigurator().apply(
        new TalonFXConfiguration().MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive));

    rotateMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ScoringConstants.rotateSoftMax).withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(ScoringConstants.rotateSoftMin));

    rotateMotor.getConfigurator()
        .apply(new FeedbackConfigs().withFeedbackRemoteSensorID(IDConstants.rotateCanCoderID)
            .withRotorToSensorRatio(ScoringConstants.rotateMotorToEncoder).withSensorToMechanismRatio(1)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));

    rotateMotor.getConfigurator()
        .apply(new SlotConfigs().withKP(ScoringConstants.rotatekP).withKI(ScoringConstants.rotatekI)
            .withKD(ScoringConstants.rotatekD).withKV(ScoringConstants.rotatekV)
            .withKG(ScoringConstants.rotatekG));

    rotateMotor.getConfigurator()
        .apply(new MotionMagicConfigs()
            .withMotionMagicAcceleration(ScoringConstants.rotateAcceleration)
            .withMotionMagicCruiseVelocity(ScoringConstants.rotateCruiseVelocity));

    rotateCanCoder = new CANcoder(IDConstants.rotateCanCoderID, "rio");

    rotateCanCoder.getConfigurator()
        .apply(new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(ScoringConstants.rotateDiscontPoint)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(ScoringConstants.rotateOffset));

    rotateCanCoder.setPosition(rotateCanCoder.getAbsolutePosition().getValue());

    coralRange = new CANrange(IDConstants.coralRangeID, "rio");

    coralRange.getConfigurator().apply(new FovParamsConfigs()
        .withFOVCenterX(0)
        .withFOVCenterY(0)
        .withFOVRangeX(6.75)
        .withFOVRangeY(6.75));

    algaeRange = new CANrange(IDConstants.algaeRangeID, "rio");

    algaeRange.getConfigurator().apply(new FovParamsConfigs()
        .withFOVCenterX(0)
        .withFOVCenterY(0)
        .withFOVRangeX(6.75)
        .withFOVRangeY(6.75));
  }

  public static void setRotateMotor(double voltage) {
    rotateMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));

  }

  public static double getRotateAngle() {
    return Units.rotationsToDegrees(
        rotateCanCoder.getAbsolutePosition().getValueAsDouble());
  }

  public static void setRotateAngle(double angle) {
    requestedRotatePos = angle;
  }

  public static void initializeRotateAngle() {
    requestedRotatePos = getRotateAngle();
  }

  public static double degreesToRotateRotations(double degrees) {
    return Units.degreesToRotations(degrees);
  }

  public static void scoringMotor(double voltage) {
    scoringMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));

  }

  public static double getCoralDistance() {
    return coralRange.getDistance().getValueAsDouble();
  }

  public static boolean coralDetected() {
    return getCoralDistance() < 0.15;
  }

  public double getAlgaeDistance() {
    return algaeRange.getDistance().getValueAsDouble();
  }

  public static void setScoringBreakMode(boolean enabled) {
    if (enabled) {
      scoringMotor.setNeutralMode(NeutralModeValue.Brake);
    } else {
      scoringMotor.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  public void log() {
    SmartDashboard.putNumber("/Scoring/RotateAngle", getRotateAngle());
    SmartDashboard.putNumber("/Scoring/TargetRotateAngle", requestedRotatePos);
    SmartDashboard.putString("/Scoring/CoralDistance", df.format(getCoralDistance()));
    SmartDashboard.putString("/Scoring/AlgaeDistance", df.format(getAlgaeDistance()));

  }

  /**
   * Servo values range from 0.0 to 1.0 corresponding to the range of full left to
   * full right.
   * 
   * @param position value Position from 0.0 to 1.0.
   * 
   */

  @Override
  public void periodic() {
    // Minimum elevator height needed for handoff
    if (getRotateAngle() < scoringRestrictedMin || requestedRotatePos < scoringRestrictedMin) {
      ElevatorSubsystem.setEnforcedMinimumHeight(true);
    } else {
      ElevatorSubsystem.setEnforcedMinimumHeight(false);
    }

    // Maximum elevator height needed for handoff
    if (getRotateAngle() < -70 || requestedRotatePos < -70) {
      ElevatorSubsystem.setEnforcedMaxHandOffHeight(true);
    } else {
      ElevatorSubsystem.setEnforcedMaxHandOffHeight(false);
    }

    double setRotatePosition = requestedRotatePos;

    // Prevent movement while elevator is under the handoff minimum
    if (ElevatorSubsystem.getElevatorPosition() < 20) {
      if (getRotateAngle() > -80) {
        setRotatePosition = Math.max(setRotatePosition, scoringRestrictedMin);
      }
    }
    if (getAlgaeDistance() < 0.25 && ElevatorSubsystem.getElevatorPosition() < 15) {
      setRotatePosition = Math.max(setRotatePosition, 70);

    }

    // Prevent movement while elevator is over the handoff maximum
    if (ElevatorSubsystem.getElevatorPosition() > 35) {
      setRotatePosition = Math.max(setRotatePosition, -50);
    }

    rotateMotor.setControl(new MotionMagicVoltage(degreesToRotateRotations(setRotatePosition)));

    log();
  }
}
