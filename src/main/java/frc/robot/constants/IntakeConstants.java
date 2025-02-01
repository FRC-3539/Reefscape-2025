package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class IntakeConstants extends BBConstants {
	public IntakeConstants() {
		super("/home/lvuser/IntakeConstants.ini", true);
		save();
	}

	public static double algaeDeployVoltage = 10.0;
	public static double algaeIntakeVoltage = 10.0;
	public static double coralDeployVoltage = 10.0;
	public static double coralIntakeVoltage = 10.0;
	public static double coralDeployMotorToEncoder = 10;
	public static double coralDeployRestingRotations = 10;
	public static double restCoralDeployAngle = 10;
	public static double homePositionAngle = 10;
	public static double groundPositionAngle = 10;
	public static double humanPositionAngle = 10;
	public static double funnelVoltage = 2;
}