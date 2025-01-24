package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class ElevatorConstants extends BBConstants {
	public ElevatorConstants() {
		super("/home/lvuser/ElevatorConstants.ini", true);
		save();
	}

	public static double elevatorVoltage = 10.0;
	public static double elevatorMotorToInches = 10.0;
	public static double troughHeight = 10.0;
	public static double lowHeight = 10.0;
	public static double midHeight = 10.0;
	public static double highHeight = 10.0;
	public static double netHeight = 10.0;
	public static double processorHeight = 10.0;
	public static double reefLowHeight = 10.0;
	public static double reefHighHeight = 10.0;
}