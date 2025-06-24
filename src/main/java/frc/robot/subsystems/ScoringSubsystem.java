// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringSubsystem extends SubsystemBase {
  DecimalFormat df = new DecimalFormat("#.00000");

  public ScoringSubsystem() {

  }

  public void log() { }

  @Override
  public void periodic() {
    log();
  }
}
