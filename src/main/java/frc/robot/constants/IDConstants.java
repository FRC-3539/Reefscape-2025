package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class IDConstants extends BBConstants {
	public IDConstants() {
		super("/home/lvuser/IDConstants.ini", true);
		save();
	}

	public static int rotateMotorID = 1;
	public static int algaeScoringMotorID = 2;
	public static int algaeDeployMotorID = 3;
	public static int algaeIntakeMotorID = 4;
	public static int coralDeployMotorID = 5;
	public static int coralIntakeMotorID = 6;
	public static int clawServoID = 1;
}