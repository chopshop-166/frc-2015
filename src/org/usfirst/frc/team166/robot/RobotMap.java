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
	public static final int FrontLeftDrivePwm = 0;
	public static final int RearLeftDrivePwm = 1;
	public static final int FrontRightDrivePwm = 2;
	public static final int RearRightDrivePwm = 3;
	public static final int ToteLiftMotorPwm = 2;// talon
	public static final int RCLiftMotorPwm = 3;// talon

	// Digital Input Channels
	public static final int CarriageRCLiftLimit = 0;
	public static final int TopLiftLimit = 1;
	public static final int BotLiftLimit = 7;
	public static final int BottomRCLiftLimit = 2;
	public static final int RCEncoderA = 3;
	public static final int RCEncoderB = 4;
	public static final int ToteEncoderA = 5;
	public static final int ToteEncoderB = 6;

	// Lift Movement
	public enum LiftMovement {
		Stopped, Up, Down
	}

}
