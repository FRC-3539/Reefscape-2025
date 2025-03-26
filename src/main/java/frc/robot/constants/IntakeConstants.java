package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class IntakeConstants extends BBConstants {
	public IntakeConstants() {
		super("/home/lvuser/IntakeConstants.ini", true);
		save();
	}

	public static double algaeIntakeVoltage = 5.0;
	public static double coralIntakeVoltage = 3.5;
	public static double funnelIntakeVoltage = 9;
	public static double funnelDeployVoltage = 2.0;
	public static double funnelDeploykP = 64.0;
	public static double funnelDeploykI = 0.0;
	public static double funnelDeploykD = 0.0;
	public static double funnelDeploykG = 0.5;
	public static double funnelDeploykV = 6.16;
	public static double funnelDeployAcceleration = 4.0;
	public static double funnelDeployCruiseVelocity = 0.8;
	public static double funnelDeploySoftMin = -40.0;
	public static double funnelDeploySoftMax = 105.0;
	public static double funnelDeployMotorToMechanism = 64.44;
	public static double funnelDeployOffset = -0.123046875;
	public static double homeFunnelDeployAngle = 100.0;
	public static double humanFunnelDeployAngle = 80.0;
	public static double groundFunnelDeployAngle = -40.0;
	public static double algaeDeployVoltage = 5.0;
	public static double algaeDeploykP = 0.0;
	public static double algaeDeploykI = 0.0;
	public static double algaeDeploykD = 0.0;
	public static double algaeDeploykG = 0.0;
	public static double algaeDeploykV = 0.0;
	public static double algaeDeployAcceleration = 0.0;
	public static double algaeDeployCruiseVelocity = 0.0;
	public static double algaeDeploySoftMin = 0.0;
	public static double algaeDeploySoftMax = 13.0;
	public static double funnelDeployDiscontPoint = 0.75;
	public static double funnelDeployRestingRotations = 0.0;
	public static double handOffFunnelDeployAngle = 80.0;
	public static int isInverted = 1;
}