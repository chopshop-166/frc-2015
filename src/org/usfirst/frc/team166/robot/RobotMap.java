package org.usfirst.frc.team166.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into to a variable name. This provides
 * flexibility changing wiring, makes checking the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// Driver Controls
	public static final int DriveJoystick = 0;
	public static final int CopilotController = 1;

	// Solenoid Channels
	public static final int ClawSolenoidForward = 0;
	public static final int ClawSolenoidReverse = 1;
	public static final int LeftWingSolenoid = 2;
	public static final int RightWingSolenoid = 3;
	public static final int ToteLiftBrakeSolenoid = 4;
	public static final int RecycleLiftBrakeSolenoid = 5;

	// PWM Channels
	public static final int FrontLeftDrivePwm = 0;
	public static final int RearLeftDrivePwm = 1;
	public static final int FrontRightDrivePwm = 2;
	public static final int RearRightDrivePwm = 3;
	public static final int ToteLiftMotorPwm = 2;
	public static final int RecyclecLiftMotorPwm = 3;
}
