package org.usfirst.frc.team166.robot;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into to a variable name. This provides
 * flexibility changing wiring, makes checking the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// Driver Controls
	public static final int DriveJoystick = 0;

	// Co Driver Controls
	public static final int ToteJoystick = 1;
	public static final int RCJoystick = 2;

	// Solenoid Channels
	public static final int ClawSolenoidForward = 0;
	public static final int ClawSolenoidReverse = 1;
	public static final int LeftWingSolenoid = 2;
	public static final int RightWingSolenoid = 3;
	public static final int ToteLiftBrakeSolenoid = 4;
	public static final int RCLiftBrakeSolenoid = 5;

	// PWM Channels
	public static final int FrontLeftDrivePwm = 2;
	public static final int RearLeftDrivePwm = 0;
	public static final int FrontRightDrivePwm = 3;
	public static final int RearRightDrivePwm = 1;
	public static final int ToteLiftMotorPwm = 4;
	public static final int RCLiftMotorPwm = 5;

	// Digital Input Channels
	public static final int FrontLeftDriveEncoderA = 4;
	public static final int FrontLeftDriveEncoderB = 5;
	public static final int RearLeftDriveEncoderA = 0;
	public static final int RearLeftDriveEncoderB = 1;
	public static final int FrontRightDriveEncoderA = 6;
	public static final int FrontRightDriveEncoderB = 7;
	public static final int RearRightDriveEncoderA = 2;
	public static final int RearRightDriveEncoderB = 3;
	public static final int CarriageRCLiftLimit = 8;
	public static final int TopLiftLimit = 9;
	public static final int BotLiftLimit = 10;
	public static final int BottomRCLiftLimit = 11;
	public static final int RCEncoderA = 12;
	public static final int RCEncoderB = 13;
	public static final int ToteEncoderA = 14;
	public static final int ToteEncoderB = 15;

	// Analog Inputs
	public static final int Gryo = 0;
	public static final int FrontRangeFinder = 1;
	public static final int RightRangeFinder = 2;
	public static final int LeftRangeFinder = 3;

}
