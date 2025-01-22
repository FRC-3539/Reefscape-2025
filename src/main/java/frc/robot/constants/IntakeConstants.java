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
}