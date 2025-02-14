package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class IntakeConstants extends BBConstants {
	public IntakeConstants() {
		super("/home/lvuser/IntakeConstants.ini", true);
		save();
	}

	public static double algaeIntakeVoltage = 10.0;
	public static double coralIntakeVoltage = 10.0;
	public static double funnelIntakeVoltage = 2;
	public static double funnelDeployVoltage = 10.0;
	public static double funnelDeploykP = 0.0;
	public static double funnelDeploykI = 0.0;
	public static double funnelDeploykD = 0.0;
	public static double funnelDeploykG = 0.0;
	public static double funnelDeploykV = 0.0;
	public static double funnelDeployAcceleration = 0.0;
	public static double funnelDeployCruiseVelocity = 0.0;
	public static double funnelDeploySoftMin = 0.0;
	public static double funnelDeploySoftMax = 0.0;
	public static double funnelDeployMotorToEncoder = 10;
	public static double funnelDeployRestingRotations = 10;
	public static double homeFunnelDeployAngle = 10;
	public static double humanFunnelDeployAngle = 10;
	public static double groundFunnelDeployAngle = 10;
	public static double algaeDeployVoltage = 10.0;
	public static double algaeDeploykP = 0.0;
	public static double algaeDeploykI = 0.0;
	public static double algaeDeploykD = 0.0;
	public static double algaeDeploykG = 0.0;
	public static double algaeDeploykV = 0.0;
	public static double algaeDeployAcceleration = 0.0;
	public static double algaeDeployCruiseVelocity = 0.0;
	public static double algaeDeploySoftMin = 0.0;
	public static double algaeDeploySoftMax = 0.0;
}