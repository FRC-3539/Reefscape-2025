package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class ClimberConstants extends BBConstants {
	public ClimberConstants() {
		super("/home/lvuser/ClimberConstants.ini", true);
		save();
	}

	public static double climberMotorToEncoder = 10.0;
	public static double climberRestingRotations = 10.0;
	public static double restClimberAngle = 10.0;
}