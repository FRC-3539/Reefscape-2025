package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class IDConstants extends BBConstants {
	public IDConstants() {
		super("/home/lvuser/IDConstants.ini", true);
		save();
	}

	public static int coralDeployCanCoderID = 9;
	public static int climberMotorID = 8;
	public static int FLSteeringID = 7;
	public static int elevatorMotorID = 7;
	public static int coralIntakeMotorID = 6;
	public static int FLDriveID = 6;
	public static int coralDeployMotorID = 5;
	public static int BLSteeringID = 5;
	public static int BLDriveID = 4;
	public static int algaeIntakeMotorID = 4;
	public static int algaeDeployMotorID = 3;
	public static int climberCanCoderID = 3;
	public static int FRSteeringID = 3;
	public static int scoringMotorID = 2;
	public static int FRDriveID = 2;
	public static int BRCanCoderID = 20;
	public static int pigeonID = 11;
	public static int rotateMotorID = 12;
	public static int BRSteeringID = 1;
	public static int BRDriveID = 0;
	public static int BLCanCoderID = 21;
	public static int FRCanCoderID = 22;
	public static int FLCanCoderID = 23;
	public static int TalonFXSID = 8;
	public static int funnelMotorID = 12;
	public static int rotateCanCoderID = 14;
}