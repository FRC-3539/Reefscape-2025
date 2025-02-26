package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class ScoringConstants extends BBConstants {
	public ScoringConstants() {
		super("/home/lvuser/ScoringConstants.ini", true);
		save();
	}

	public static double algaeScoringVoltage = -2.5;
	public static double coralScoringVoltage = 2.5;
	public static double rotateVoltage = 2;
	public static double rotatekP = 100;
	public static double rotatekI = 0.0;
	public static double rotatekD = 0.0;
	public static double rotatekG = 0.0;
	public static double rotatekV = 19.44;
	public static double rotateAcceleration = 3;
	public static double rotateCruiseVelocity = .4;
	public static double rotateMotorToInches = 0.0;
	public static double restRotateAngle = -135;
	public static double rotateRestingRotations = 0.0;
	public static double rotateMotorToEncoder = 162;
	public static double rotateSoftMin = -0.38;
	public static double rotateSoftMax = 0.25;
	public static double rotateOffset = -0.542;
	public static double troughPosition = 70;
	public static double coralLowPosition = 40;
	public static double coralMidPosition = 43;
	public static double coralHighPosition = 15;
	public static double processorPosition = 80;
	public static double netPosition = 90;
	public static double algaeLowPosition = 50;
	public static double algaeHighPosition = 50;
	public static double rotateDiscontPoint = 0.3;
	public static double handOffPosition = -130;
	public static double groundPosition = 40;
}