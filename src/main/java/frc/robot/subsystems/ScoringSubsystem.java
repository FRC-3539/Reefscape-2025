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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    algaeRange = new CANrange(26, "rio");
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
    shootingMotor.setNeutralMode(NeutralModeValue.Brake);

    rotateCanCoder = new CANcoder(IDConstants.rotateCanCoderID, "rio");

    rotateCanCoder.getConfigurator()
    .apply(new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(ScoringConstants.rotateDiscontPoint)
    .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
    .withMagnetOffset(ScoringConstants.rotateOffset));


    shootingMotor.clearStickyFault_DeviceTemp();
    shootingMotor.clearStickyFault_ProcTemp();
  }


  public static void setRotateAngle(double angle) { requestRotatePos = angle;}

  public static double getRotateAngle(){return Units.rotationsToDegrees(rotateCanCoder.getAbsolutePosition().getValueAsDouble());}
  public static double getCoralDistance() {
    return coralRange.getDistance().getValueAsDouble();
  }
  public static double getAlgaeDistance() {
    return algaeRange.getDistance().getValueAsDouble();
  }
  public static void setShootingMotor(double voltage) {
    shootingMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public void log() {
    SmartDashboard.putNumber("/Scoring/ScoringMotor_DeviceTemp", 
    shootingMotor.getDeviceTemp().getValue().magnitude());
    SmartDashboard.putNumber("/Scoring/ScoringMotor_ProcessorTemp",
    shootingMotor.getProcessorTemp().getValue().magnitude());

    SmartDashboard.putBoolean("/Scoring/ScoringMotor_StickyMotor_DeviceTemp",
  shootingMotor.getStickyFault_DeviceTemp().getValue());
  SmartDashboard.putBoolean("/Scoring/ScoringMotor_StickyProcessorTemp",
    shootingMotor.getStickyFault_ProcTemp().getValue());
  }
  //get Requested Arm Position
  public static double getRequestedArmPosition() {
  return requestRotatePos;

  }
  private static double scoringRestrictedMin = 30;

  @Override
  public void periodic() {
    log();
    
// Minimum elevator height needed for handoff
if (getRotateAngle() < scoringRestrictedMin || requestRotatePos < scoringRestrictedMin) {
  ElevatorSubsystem.setEnforcedMinimumHeight(true);
} else {
  ElevatorSubsystem.setEnforcedMinimumHeight(false);
}

// Maximum elevator height needed for handoff
if (getRotateAngle() < -70 || requestRotatePos < -70) {
  ElevatorSubsystem.setEnforcedMaxHandOffHeight(true);
} else {
  ElevatorSubsystem.setEnforcedMaxHandOffHeight(false);
}

double setRotatePosition = requestRotatePos;

// Prevent movement while elevator is under the handoff minimum
if (ElevatorSubsystem.getElevatorPosition() < 20) {
  if (getRotateAngle() > -80) {
    setRotatePosition = Math.max(setRotatePosition, scoringRestrictedMin);
  }
}

// Flip arm up if algae detected
// if (algaeDetected()
//     && (ElevatorSubsystem.getElevatorPosition() < 15 || (ElevatorSubsystem.requestedElevatorPos < 15))) {
//   setRotatePosition = Math.max(setRotatePosition, 70);

// }

// Prevent movement while elevator is over the handoff maximum
if (ElevatorSubsystem.getElevatorPosition() > 35) {
  setRotatePosition = Math.max(setRotatePosition, -50);
}
    
  rotateMotor.setControl(new MotionMagicVoltage(Units.degreesToRotations(setRotatePosition)));

  }
}
