// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringSubsystem extends SubsystemBase {
  DecimalFormat df = new DecimalFormat("#.00000");

  private static TalonFX shootingMotor;
  private static CANrange algaeRange, coralRange;

  public ScoringSubsystem() {
     shootingMotor = new TalonFX(15, "rio");
     coralRange = new CANrange(18, "rio");
    
  }

  public static double getCoralDistance() {
    return coralRange.getDistance().getValueAsDouble();
  }
public static void setShootingMotor(double voltage) {
    shootingMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public void log() { }

  @Override
  public void periodic() {
    log();
  }
}
