// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IDConstants;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;


public class IntakeSubsystem extends SubsystemBase { 
  
  private static TalonFX algaeDeployMotor,algaeIntakeMotor,funnelDeployMotor, funnelIntakeMotor, coralIntakeMotor;
 
  private static CANcoder funnelDeployCanCoder;
  private static double requestedFunnelDeployPos = 0;

  public IntakeSubsystem() {

    funnelDeployCanCoder = new CANcoder(IDConstants.funnelDeployCanCoderID, "rio");

    algaeDeployMotor = new TalonFX(IDConstants.algaeDeployMotorID, "rio");
    algaeDeployMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive));
  
    algaeIntakeMotor = new TalonFX(IDConstants.algaeIntakeMotorID, "rio");
    algaeIntakeMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive));
  
    funnelDeployMotor = new TalonFX(IDConstants.funnelDeployMotorID, "rio");
    funnelDeployMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive));
  
    coralIntakeMotor = new TalonFX(IDConstants.coralIntakeMotorID, "rio");
    coralIntakeMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive));

    funnelIntakeMotor = new TalonFX(IDConstants.funnelIntakeMotorID, "rio");
    funnelIntakeMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
          .withInverted(InvertedValue.CounterClockwise_Positive));

    algaeDeployMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(IntakeConstants.algaeDeploySoftMax).withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(IntakeConstants.algaeDeploySoftMin));

    algaeDeployMotor.getConfigurator()
				.apply(new SlotConfigs().withKP(IntakeConstants.algaeDeploykP).withKI(IntakeConstants.algaeDeploykI)
						.withKD(IntakeConstants.algaeDeploykD).withKV(IntakeConstants.algaeDeploykV)
						.withKG(IntakeConstants.algaeDeploykG));
    
    algaeDeployMotor.getConfigurator()
				.apply(new MotionMagicConfigs()
        .withMotionMagicAcceleration(IntakeConstants.algaeDeployAcceleration)
        .withMotionMagicCruiseVelocity(IntakeConstants.algaeDeployCruiseVelocity));

    funnelDeployMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
				.withForwardSoftLimitThreshold(IntakeConstants.funnelDeploySoftMax).withReverseSoftLimitEnable(true)
				.withReverseSoftLimitThreshold(IntakeConstants.funnelDeploySoftMin));

    funnelDeployMotor.getConfigurator()
				.apply(new FeedbackConfigs().withFeedbackRemoteSensorID(IDConstants.funnelDeployCanCoderID)
						.withRotorToSensorRatio(IntakeConstants.funnelDeployMotorToEncoder).withSensorToMechanismRatio(1)
						.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));

    funnelDeployMotor.getConfigurator()
				.apply(new SlotConfigs().withKP(IntakeConstants.funnelDeploykP).withKI(IntakeConstants.funnelDeploykI)
						.withKD(IntakeConstants.funnelDeploykD).withKV(IntakeConstants.funnelDeploykV)
						.withKG(IntakeConstants.funnelDeploykG).withGravityType(GravityTypeValue.Arm_Cosine));
            
    funnelDeployMotor.getConfigurator()
            .apply(new MotionMagicConfigs()
            .withMotionMagicAcceleration(IntakeConstants.funnelDeployAcceleration)
            .withMotionMagicCruiseVelocity(IntakeConstants.funnelDeployCruiseVelocity));
  }
    public static double getFunnelDeployAngle() {
		return Units.rotationsToDegrees(
				funnelDeployCanCoder.getAbsolutePosition().getValueAsDouble() - IntakeConstants.funnelDeployRestingRotations)
				+ IntakeConstants.homeFunnelDeployAngle;
  }

  public static void setFunnelDeployAngle(double angle) {
		requestedFunnelDeployPos = angle;
	}

  public static void initializFunnelDeployAngle() {
		requestedFunnelDeployPos = getFunnelDeployAngle();
	}
  
  public static void setAlgaeIntakeMotor(double voltage) {
    algaeIntakeMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public static void setAlgaeDeployMotor(double voltage) {
    algaeDeployMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public static void setfunnelDeployMotor(double voltage) {
    funnelDeployMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public static void setCoralIntakeMotor(double voltage) {
    coralIntakeMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }
  public static void setFunnelIntakeMotor(double voltage) {
    funnelIntakeMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public static double degreesToFunnelDeployRotations(double degrees) {
		return Units.degreesToRotations(degrees - IntakeConstants.homeFunnelDeployAngle)
				+ IntakeConstants.funnelDeployRestingRotations;
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    funnelDeployMotor.setControl(new MotionMagicVoltage(degreesToFunnelDeployRotations(requestedFunnelDeployPos)));
  }
}
