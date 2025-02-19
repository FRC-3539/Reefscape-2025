package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class ElevatorConstants extends BBConstants {
	public ElevatorConstants() {
		super("/home/lvuser/ElevatorConstants.ini", true);
		save();
	}

	public static double elevatorVoltage = 2;
	public static double elevatorInchesPerRotation = 0.2834;
	public static double troughHeight = 7;
	public static double coralLowHeight = 22.75;
	public static double coralMidHeight = 38.75;
	public static double coralHighHeight = 22.75;
	public static double netHeight = 22.75;
	public static double processorHeight = 22.75;
	public static double algaeLowHeight = 22.75;
	public static double algaeHighHeight = 22.75;
	public static double elevatorkP = 10;
	public static double elevatorkI = 0.0;
	public static double elevatorkD = 0.0;
	public static double elevatorkG = 0.0;
	public static double elevatorkV = 0.0;
	public static double elevatorAcceleration = 5;
	public static double elevatorCruiseVelocity = 17;
	public static double elevatorSoftMin = 5;
	public static double elevatorSoftMax = 60;
	public static double elevatorHomePositionOffset = 22.75;
}