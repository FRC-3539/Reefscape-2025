// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  // Declare system parts here
  private static TalonFXS outerfunnelmoter, innerfunnelmotor;

  public IntakeSubsystem() {
    // Initialize system parts here
    outerfunnelmoter=new TalonFXS(8,"rio");
    innerfunnelmotor=new TalonFXS(10,"rio");
  }

  // Setters / getters
public static void setouterfunnelmoter(double voltage){
  outerfunnelmoter.setControl(new VoltageOut(voltage).withEnableFOC(true));

}
public static void setinnerfunnelmotor(double voltage){
  innerfunnelmotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
}

  public void log() { }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }
}

