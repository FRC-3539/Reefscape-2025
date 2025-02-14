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
	public static double climberVoltage = 0.0;
	public static double climberkP = 0.0;
	public static double climberkI = 0.0;
	public static double climberkD = 0.0;
	public static double climberkG = 0.0;
	public static double climberkV = 0.0;
	public static double climberAcceleration = 0.0;
	public static double climberCruiseVelocity = 0.0;
	public static double climberSoftMin = 0.0;
	public static double climberSoftMax = 0.0;
}