package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class ElevatorConstants extends BBConstants {
	public ElevatorConstants() {
		super("/home/lvuser/ElevatorConstants.ini", true);
		save();
	}

	public static double elevatorVoltage = 2;
	public static double elevatorInchesPerRotation = 0.7872;
	public static double troughHeight = 7;
	public static double coralLowHeight = 22.25;
	public static double coralMidHeight = 37.75;
	public static double coralHighHeight = 74.75;
	public static double netHeight = 79.75;
	public static double processorHeight = 22.75;
	public static double algaeLowHeight = 33.5;
	public static double algaeHighHeight = 49.0;
	public static double elevatorkP = 10;
	public static double elevatorkI = 0.0;
	public static double elevatorkD = 0.0;
	public static double elevatorkG = 0.0;
	public static double elevatorkV = 0;
	public static double elevatorAcceleration = 150;
	public static double elevatorCruiseVelocity = 90; // max 100
	public static double elevatorSoftMin = 5;
	public static double elevatorSoftMax = 80;
	public static double elevatorHomePositionOffset = 22.75;
	public static double handOffHeight = 22.75;
	public static double groundHeight = 8;
}