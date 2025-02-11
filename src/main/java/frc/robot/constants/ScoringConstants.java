package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class ScoringConstants extends BBConstants {
	public ScoringConstants() {
		super("/home/lvuser/ScoringConstants.ini", true);
		save();
	}

	public static double rotateVoltage = 10.0;
	public static double algaeScoringVoltage = 10.0;
	public static double rotateMotorToInches = 0.0;
	public static double troughPosition = 0.0;
	public static double lowPosition = 0.0;
	public static double midPosition = 0.0;
	public static double highPosition = 0.0;
	public static double processorPosition = 0.0;
	public static double netPosition = 0.0;
	public static double reefLowPosition = 0.0;
	public static double reefHighPosition = 0.0;
	public static double rotateMotorToEncoder = 0.0;
	public static double rotateRestingRotations = 0.0;
	public static double restRotateAngle = 0.0;
	public static double coralScoringVoltage = 10.0;
}