// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class ScoringSubsystem extends SubsystemBase {
  private static TalonFX rotateMotor, algaeScoringMotor;
  private static Servo clawServo;
  private static double clawServoPosition = 0;


  public ScoringSubsystem() {
    rotateMotor = new TalonFX(IDConstants.rotateMotorID, "rio");
    rotateMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive));

    algaeScoringMotor = new TalonFX(IDConstants.algaeScoringMotorID, "rio");
    algaeScoringMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive));

    clawServo = new Servo(IDConstants.clawServoID);

  }
  public static void setRotateMotor(double voltage){
        rotateMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));

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
  }
}
