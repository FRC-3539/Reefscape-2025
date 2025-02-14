package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class ElevatorConstants extends BBConstants {
	public ElevatorConstants() {
		super("/home/lvuser/ElevatorConstants.ini", true);
		save();
	}

	public static double elevatorVoltage = 2;
	public static double elevatorMotorToInches = 0;
	public static double troughHeight = 0;
	public static double coralLowHeight = 0;
	public static double coralMidHeight = 0;
	public static double coralHighHeight = 0;
	public static double netHeight = 0;
	public static double processorHeight = 0;
	public static double algaeLowHeight = 0;
	public static double algaeHighHeight = 0;
	public static double elevatorkP = 0.0;
	public static double elevatorkI = 0.0;
	public static double elevatorkD = 0.0;
	public static double elevatorkG = 0.0;
	public static double elevatorkV = 0.0;
	public static double elevatorAcceleration = 0.0;
	public static double elevatorCruiseVelocity = 0.0;
	public static double elevatorSoftMin = 0.0;
	public static double elevatorSoftMax = 0.0;
}