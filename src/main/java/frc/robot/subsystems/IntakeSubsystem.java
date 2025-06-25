// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private static TalonFXS outerFunnelMotor, innerFunnelMotor;

  public IntakeSubsystem() {
    outerFunnelMotor = new TalonFXS(8, "rio");
    innerFunnelMotor = new TalonFXS(10, "rio");
  }

  public static void setOuterFunnelMotor(double voltage) {
    outerFunnelMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }
  public static void setInnerFunnelMotor(double voltage) {
    innerFunnelMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }
  public void log() { }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }
}

