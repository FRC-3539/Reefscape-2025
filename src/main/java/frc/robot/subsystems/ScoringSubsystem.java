// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IDConstants;
import frc.robot.constants.ScoringConstants;

public class ScoringSubsystem extends SubsystemBase {
  DecimalFormat df = new DecimalFormat("#.00000");

  private static TalonFX shootingMotor, rotateMotor;
  private static CANrange algaeRange, coralRange;
  private static CANcoder rotateCanCoder;
  private static double requestRotatePos = 0;

  public ScoringSubsystem() {
    shootingMotor = new TalonFX(15, "rio");
    coralRange = new CANrange(18, "rio");
    rotateMotor = new TalonFX(14, "rio");
    rotateMotor.getConfigurator()
        .apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ScoringConstants.rotateSoftMax).withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ScoringConstants.rotateSoftMin));
    rotateMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
    rotateMotor.setNeutralMode(NeutralModeValue.Brake);
    rotateMotor.getConfigurator()
        .apply(new SlotConfigs().withKP(ScoringConstants.rotatekP).withKI(ScoringConstants.rotatekI)
            .withKD(ScoringConstants.rotatekD).withKG(ScoringConstants.rotatekG).withKV(ScoringConstants.rotatekV));

    rotateMotor.getConfigurator()
        .apply(new MotionMagicConfigs().withMotionMagicAcceleration(ScoringConstants.rotateAcceleration)
            .withMotionMagicCruiseVelocity(ScoringConstants.rotateCruiseVelocity));
    rotateMotor.getConfigurator()
    .apply(new FeedbackConfigs().withFeedbackRemoteSensorID(IDConstants.rotateCanCoderID)
    .withRotorToSensorRatio(ScoringConstants.rotateMotorToEncoder).withSensorToMechanismRatio(1)
    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));

    rotateCanCoder = new CANcoder(IDConstants.rotateCanCoderID, "rio");

    rotateCanCoder.getConfigurator()
    .apply(new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(ScoringConstants.rotateDiscontPoint)
    .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
    .withMagnetOffset(ScoringConstants.rotateOffset));

  }


  public static void setRotateAngle(double angle) { requestRotatePos = angle;}

  public static double getRotateAngle(){return Units.rotationsToDegrees(rotateCanCoder.getAbsolutePosition().getValueAsDouble());}
  public static double getCoralDistance() {
    return coralRange.getDistance().getValueAsDouble();
  }

  public static void setShootingMotor(double voltage) {
    shootingMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public void log() {
  }

  @Override
  public void periodic() {
    log();
  
  rotateMotor.setControl(new MotionMagicVoltage(Units.degreesToRotations(requestRotatePos)));

  }
}
