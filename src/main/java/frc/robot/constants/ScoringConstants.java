package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class ScoringConstants extends BBConstants {
	public ScoringConstants() {
		super("/home/lvuser/ScoringConstants.ini", true);
		save();
	}

	public static double rotateVoltage = 10.0;
	public static double algaeScoringVoltage = 10.0;
	public static double clawOpenPosition = 10.0;
	public static double clawClosedPosition = 0.0;
}