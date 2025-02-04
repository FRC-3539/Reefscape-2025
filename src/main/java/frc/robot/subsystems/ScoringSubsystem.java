// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ScoringConstants;
import frc.robot.constants.IDConstants;

public class ScoringSubsystem extends SubsystemBase {
  private static TalonFX rotateMotor, algaeScoringMotor;
  private static CANcoder rotateCanCoder;
  private static Servo clawServo;
  private static double clawServoPosition = 0;
  private static double requestedRotatePos = 0;


  public ScoringSubsystem() {
    rotateMotor = new TalonFX(IDConstants.rotateMotorID, "rio");
    rotateMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive));

     rotateMotor.getConfigurator()
				.apply(new FeedbackConfigs().withFeedbackRemoteSensorID(IDConstants.rotateCanCoderID)
						.withRotorToSensorRatio(ScoringConstants.rotateMotorToEncoder).withSensorToMechanismRatio(1)
						.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));

    algaeScoringMotor = new TalonFX(IDConstants.algaeScoringMotorID, "rio");
    algaeScoringMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive));

    clawServo = new Servo(IDConstants.clawServoID);

    rotateCanCoder = new CANcoder(IDConstants.rotateCanCoderID, "rio");
  }
  public static void setRotateMotor(double voltage){
        rotateMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));

  }
  public static double getRotateAngle() {
		return Units.rotationsToDegrees(
				rotateCanCoder.getAbsolutePosition().getValueAsDouble() - ScoringConstants.rotateRestingRotations)
				+ ScoringConstants.restRotateAngle;
  }

  public static void setRotateAngle(double angle) {
		requestedRotatePos = angle;
	}

  public static void initializRotateAngle() {
		requestedRotatePos = getRotateAngle();
	}
  
  public static double degreesToRotateRotations(double degrees) {
		return Units.degreesToRotations(degrees - ScoringConstants.restRotateAngle)
				+ ScoringConstants.rotateRestingRotations;
	}
  
  public static void setAlgaeScoringMotor(double voltage){
    algaeScoringMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));

}
  public static void setClawServoPosition(double position) {
		clawServoPosition = position;
	}

  

  @Override
  public void periodic() {
    clawServo.set(clawServoPosition);
    rotateMotor.setControl(new MotionMagicVoltage(degreesToRotateRotations(requestedRotatePos)));

    }
  }

