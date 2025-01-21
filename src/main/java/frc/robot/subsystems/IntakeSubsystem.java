// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;


public class IntakeSubsystem extends SubsystemBase { 
  
  private TalonFX algaeDeployMotor,algaeIntakeMotor,coralDeployMotor,coralIntakeMotor;
 

  public IntakeSubsystem() {

    algaeDeployMotor = new TalonFX(1, "rio");
    algaeDeployMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
  
    algaeIntakeMotor = new TalonFX(2, "rio");
    algaeIntakeMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
  
    coralDeployMotor = new TalonFX(3, "rio");
    coralDeployMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
  
    coralIntakeMotor = new TalonFX(4, "rio");
    coralIntakeMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
  }
  
  public void setAlgaeIntakeMotor(double voltage) {
    algaeIntakeMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public void setAlgaeDeployMotor(double voltage) {
    algaeDeployMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public void setCoralDeployMotor(double voltage) {
    coralDeployMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public void setCoralIntakeMotor(double voltage) {
    coralIntakeMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
