package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class DriveConstants extends BBConstants {
	public DriveConstants() {
		super("/home/lvuser/DriveConstants.ini", true);
		save();
	}

	public static double TranslationkP = 5;
	public static double TranslationkI = 0.0;
	public static double TranslationkD = 0.0;
	public static double RotationkP = 10.0;
	public static double RotationkI = 0.0;
	public static double RotationkD = 0.0;
	public static double TranslationkV = 1.0;
	public static double TranslationkA = 0.0;
	public static double TranslationkS = 0.0;
	public static double FLSteerOffset = 0.0;
	public static double FRSteerOffset = 0.0;
	public static double BLSteerOffset = 0.0;
	public static double BRSteerOffset = 0.0;
	public static double speedMultiplier = 0.7;
	public static double rotationSpeedMultiplier = 0.35;
	public static double turboSpeedMultiplier = 1.0;
	public static double turboRotationSpeedMultiplier = 0.55;
	public static double AlignkP = 2.0;
}