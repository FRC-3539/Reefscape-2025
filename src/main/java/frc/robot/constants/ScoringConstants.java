package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class ScoringConstants extends BBConstants {
	public ScoringConstants() {
		super("/home/lvuser/ScoringConstants.ini", true);
		save();
	}

	public static double algaeScoringVoltage = 2;
	public static double coralScoringVoltage = 2;
	public static double rotateVoltage = 2;
	public static double rotatekP = 0.0;
	public static double rotatekI = 0.0;
	public static double rotatekD = 0.0;
	public static double rotatekG = 0.0;
	public static double rotatekV = 0.0;
	public static double rotateAcceleration = 0.0;
	public static double rotateCruiseVelocity = 0.0;
	public static double rotateMotorToInches = 0.0;
	public static double restRotateAngle = 0.0;
	public static double rotateRestingRotations = 0.0;
	public static double rotateMotorToEncoder = 0.0;
	public static double troughPosition = 0.0;
	public static double coralLowPosition = 0.0;
	public static double coralMidPosition = 0.0;
	public static double coralHighPosition = 0.0;
	public static double processorPosition = 0.0;
	public static double netPosition = 0.0;
	public static double algaeLowPosition = 0.0;
	public static double algaeHighPosition = 0.0;
	public static double rotateSoftMin = 0.0;
	public static double rotateSoftMax = 0.0;
}