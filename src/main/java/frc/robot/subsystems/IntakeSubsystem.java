// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ScoringConstants;

public class IntakeSubsystem extends SubsystemBase {
  // Declare system parts here
  private static TalonFXS outerfunnelmoter, innerfunnelmotor;
  private static TalonFX funnelDeployMotor;

  private static CANcoder funnelDeployCanCoder;
  private static double requestFunnelPos =0;
  public IntakeSubsystem() {
    // Initialize system parts here
    outerfunnelmoter = new TalonFXS(8, "rio");
    innerfunnelmotor = new TalonFXS(10, "rio");
    funnelDeployCanCoder = new CANcoder(24, "rio");
    

    funnelDeployCanCoder.getConfigurator()
        .apply(new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(IntakeConstants.funnelDeployDiscontPoint)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withMagnetOffset(IntakeConstants.funnelDeployOffset));

    funnelDeployMotor = new TalonFX(9, "rio");

    funnelDeployMotor.getConfigurator()
        .apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(IntakeConstants.funnelDeploySoftMax).withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(IntakeConstants.funnelDeploySoftMin));
    funnelDeployMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
    funnelDeployMotor.setNeutralMode(NeutralModeValue.Brake);
    funnelDeployMotor.getConfigurator()
        .apply(new SlotConfigs().withKP(IntakeConstants.funnelDeploykP).withKI(IntakeConstants.funnelDeploykI)
            .withKD(IntakeConstants.funnelDeploykD).withKG(IntakeConstants.funnelDeploykG)
            .withKV(IntakeConstants.funnelDeploykV).withGravityType(GravityTypeValue.Arm_Cosine));

    funnelDeployMotor.getConfigurator()
        .apply(new MotionMagicConfigs().withMotionMagicAcceleration(IntakeConstants.funnelDeployAcceleration)
            .withMotionMagicCruiseVelocity(IntakeConstants.funnelDeployCruiseVelocity));
    funnelDeployMotor.getConfigurator()
        .apply(new FeedbackConfigs().withFeedbackRemoteSensorID(IDConstants.funnelDeployCanCoderID)
            .withRotorToSensorRatio(IntakeConstants.funnelDeployMotorToMechanism).withSensorToMechanismRatio(1)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));

  }

  // Setters / getters
  public static void setouterfunnelmoter(double voltage) {
    outerfunnelmoter.setControl(new VoltageOut(voltage).withEnableFOC(true));}
public static void setFunnelDeployAngle(double angle) { requestFunnelPos = angle;

  }

  public static void setinnerfunnelmotor(double voltage) {
    innerfunnelmotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public void log() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
    funnelDeployMotor.setControl(new MotionMagicVoltage(Units.degreesToRotations(requestFunnelPos)));
  }
}