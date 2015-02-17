package org.usfirst.frc.team166.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into to a variable name. This provides
 * flexibility changing wiring, makes checking the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// Driver Controls
	public static final int DriveJoystick = 0;
	public static final int DriveJoystickTwistAxis = 2;

	// Co Driver Controls
	public static final int CopilotController = 1;
	public static final int RcLiftUpDownAxis = 5;
	public static final int ToteLiftUpDownAxis = 1;
	public static final int RightXboxTrigger = 3;
	public static final int XboxBButton = 1;

	// Solenoid Channels
	public static class solenoid {
		public static final int Pcm24 = 1;
		public static final int Pcm12 = 0;
		public static final int ClawForward = 5;
		public static final int ClawReverse = 4;
		public static final int LeftWingForward = 0;
		public static final int LeftWingReverse = 1;
		public static final int RightWingForward = 2;
		public static final int RightWingReverse = 3;
		public static final int ToteLiftBrakeForward = 2;
		public static final int ToteLiftBrakeReverse = 3;
		public static final int RCLiftBrakeForward = 0;
		public static final int RCLiftBrakeReverse = 1;
	}

	// PWM Channels
	public static class Pwm {
		public static final int FrontLeftDrive = 1;
		public static final int RearLeftDrive = 2;
		public static final int FrontRightDrive = 0;
		public static final int RearRightDrive = 3;
		public static final int ToteLiftMotor = 4;
		public static final int RCLiftMotor = 5;
	}

	// Encoder (Digital Input) Channels
	public static class Encoders {
		public static final int FrontLeftDriveA = 12;
		public static final int FrontLeftDriveB = 13;
		public static final int RearLeftDriveA = 16;
		public static final int RearLeftDriveB = 17;
		public static final int FrontRightDriveA = 10;
		public static final int FrontRightDriveB = 11;
		public static final int RearRightDriveA = 14;
		public static final int RearRightDriveB = 15;
		public static final int RCLiftA = 18;
		public static final int RCLiftB = 19;
		public static final int ToteLiftA = 22;
		public static final int ToteLiftB = 23;
	}

	public static class Switch {
		public static final int CarriageRCLiftLimit = 7;
		public static final int LiftUpperLimit = 8;
		public static final int LiftLowerLimit = 9;
	}

	// Analog Inputs
	public static class Analog {
		public static final int Gryo = 0;
		public static final int FrontRangeFinder = 1;
		public static final int RightRangeFinder = 2;
		public static final int LeftRangeFinder = 3;
	}

	// Preference Strings
	public static class Prefs {
		public static final String DriveDeadZone = "DriveDeadZone";
		public static final String DriveScalerX = "DriveScalerX";
		public static final String DriveScalerY = "DriveScalerY";
		public static final String DriveScalerRotation = "DriveScalerRotation";
		public static final String GyroStrafeConstant = "GyroStrafeConstant";
		public static final String AutoForwardDistance1 = "AutoForwardDistance1";
		public static final String AutoForwardDistance2 = "AutoForwardDistance2";
		public static final String AutoForwardSpeed = "AutoForwardSpeed";
		public static final String AutoDriveSpeed = "AutoDriveSpeed";
		public static final String AutoDesiredDistanceToWall = "AutoDesiredDistanceToWall";
		public static final String StrafeSpeed = "StrafeSpeed";
		public static final String CenterDistanceConstant = "CenterDistanceConstant";
		public static final String StalledDriveCurrent = "StalledDriveCurrent";
		public static final String DriveSpeedP = "DriveSpeedP";
		public static final String DriveSpeedI = "DriveSpeedI";
		public static final String DriveSpeedD = "DriveSpeedD";
		public static final String DriveSpeedF = "DriveSpeedF";

		public static final String LiftEncoderMin = "LiftEncoderMin";
		public static final String LiftDeadzone = "LiftDeadzone";
		public static final String LiftSpeed = "LiftSpeed";
		public static final String LiftMaxCurrent = "LiftMaxCurrent";
		public static final String LiftDistPerPulse = "LiftDistPerPulse";
		public static final String LiftSpeedP = "LiftSpeedP";
		public static final String LiftSpeedI = "LiftSpeedI";
		public static final String LiftSpeedD = "LiftSpeedD";
		public static final String LiftSpeedF = "LiftSpeedF";
		public static final String RightXboxTriggerDeadzone = "RightXboxTriggerDeadzone";

		public static final String LiftPosTolerance = "LiftPosTolerance";

		public static final String RightRCDirection = "RightRCDirection";
		public static final String RightRCWaitTime = "RightRCWaitTime";
		public static final String RCRightLiftPos = "RCRightLiftPos";
		public static final String RCRightToteLiftPos = "RCRightToteLiftPos";
	}

	// PD Circuit Channels
	public class Power {
		public static final int FrontLeftDrive = 2;
		public static final int RearLeftDrive = 14;
		public static final int FrontRightDrive = 0;
		public static final int RearRightDrive = 15;
		public static final int ToteLiftMotor = 2;
		public static final int RCLiftMotor = 1;
	}
}
