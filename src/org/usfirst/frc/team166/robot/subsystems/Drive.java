package org.usfirst.frc.team166.robot.subsystems;

//IMPORTS
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team166.robot.PIDSpeedController;
import org.usfirst.frc.team166.robot.RobotMap;
import org.usfirst.frc.team166.robot.commands.DriveWithJoysticks;

public class Drive extends Subsystem {
	// ROBOTDRIVE INIT
	final RobotDrive robotDrive;

	// PDP INIT
	PowerDistributionPanel pdp;

	// RANGEFINDER INITS
	final AnalogInput frontRangefinder;
	final AnalogInput leftRangefinder;
	final AnalogInput rightRangefinder;

	// PID CONTROLLER INITS
	final PIDSpeedController frontLeftPID;
	final PIDSpeedController frontRightPID;
	final PIDSpeedController rearLeftPID;
	final PIDSpeedController rearRightPID;

	// TALON INITS
	final Talon frontLeftTalon;
	final Talon rearLeftTalon;
	final Talon frontRightTalon;
	final Talon rearRightTalon;

	// ENCODER INITS
	final Encoder frontLeftEncoder;
	final Encoder frontRightEncoder;
	final Encoder rearLeftEncoder;
	final Encoder rearRightEncoder;

	// YAWRATE SENSOR INIT
	public final Gyro gyro;

	// YAWRATE SENSOR VARIABLES
	double gyroOffset;

	// ULTRASONIC VARIABLES
	double frontUSDistance;
	double rightUSDistance;
	double leftUSDistance;
	double USConstant = 102.0408163265306;
	double centerOffsetDistance = 0;

	// DRIVE VARIABLES
	boolean driveMode = false;
	double joystickScalerX;
	double joystickScalerY;
	double joystickScalerRotation;

	public Drive() {

		// POWER DISTRIBUTION PANEL
		pdp = new PowerDistributionPanel();

		// SPEED CONTROLLER PORTS
		frontLeftTalon = new Talon(RobotMap.FrontLeftDrivePwm);
		rearLeftTalon = new Talon(RobotMap.RearLeftDrivePwm);
		frontRightTalon = new Talon(RobotMap.FrontRightDrivePwm);
		rearRightTalon = new Talon(RobotMap.RearRightDrivePwm);

		// ULTRASONIC SENSORS
		frontRangefinder = new AnalogInput(RobotMap.FrontRangeFinder);
		rightRangefinder = new AnalogInput(RobotMap.RightRangeFinder);
		leftRangefinder = new AnalogInput(RobotMap.LeftRangeFinder);

		// YAWRATE SENSOR
		gyro = new Gyro(RobotMap.Gryo);

		// ENCODER PORTS
		frontLeftEncoder = new Encoder(RobotMap.FrontLeftDriveEncoderA, RobotMap.FrontLeftDriveEncoderB);
		rearLeftEncoder = new Encoder(RobotMap.RearLeftDriveEncoderA, RobotMap.RearLeftDriveEncoderB);
		frontRightEncoder = new Encoder(RobotMap.FrontRightDriveEncoderA, RobotMap.FrontRightDriveEncoderB);
		rearRightEncoder = new Encoder(RobotMap.RearRightDriveEncoderA, RobotMap.RearRightDriveEncoderB);

		// ENCODER MATH
		frontLeftEncoder.setDistancePerPulse(((6 * Math.PI) / 1024) / 183);
		frontLeftEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);
		frontRightEncoder.setDistancePerPulse(((6 * Math.PI) / 1024) / 183);
		frontRightEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);
		rearLeftEncoder.setDistancePerPulse(((6 * Math.PI) / 1024) / 183);
		rearLeftEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);
		rearRightEncoder.setDistancePerPulse(((6 * Math.PI) / 1024) / 183);
		rearRightEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);

		// PID SPEED CONTROLLERS
		frontLeftPID = new PIDSpeedController(frontLeftEncoder, frontLeftTalon, "Drive", "frontLeft");
		frontRightPID = new PIDSpeedController(frontRightEncoder, frontRightTalon, "Drive", "frontRight");
		rearLeftPID = new PIDSpeedController(rearLeftEncoder, rearLeftTalon, "Drive", "rearLeft");
		rearRightPID = new PIDSpeedController(rearRightEncoder, rearRightTalon, "Drive", "rearRight");

		// DRIVE DECLARATION
		robotDrive = new RobotDrive(frontLeftPID, rearLeftPID, frontRightPID, rearRightPID);
		robotDrive.setExpiration(0.1);

		// MOTOR INVERSIONS
		robotDrive.setInvertedMotor(MotorType.kFrontRight, true);
		robotDrive.setInvertedMotor(MotorType.kRearRight, true);
	}

	// MECANUMDRIVE USING DEADZONES, GYRO, AND ENCODERS
	public void mecanumDrive(Joystick stick) {
		printPDPAverageCurrent();
		joystickScalerX = Preferences.getInstance().getDouble("joystickScalerX", 1);
		joystickScalerY = Preferences.getInstance().getDouble("joystickScalerY", 1);
		joystickScalerRotation = Preferences.getInstance().getDouble("joystickScalerRotation", 1);
		// DRIVE FORWARD WITH JOYSTICKS ONLY
		if ((Math.abs(stick.getX()) > .1) || (Math.abs(stick.getY()) > .1) || (Math.abs(stick.getRawAxis(3)) > .1)) {
			if (Math.abs(stick.getRawAxis(3)) > .1) {
				robotDrive.mecanumDrive_Cartesian(stick.getX() * joystickScalerX, stick.getY() * joystickScalerY,
						stick.getRawAxis(3) * joystickScalerRotation, 0);
				driveMode = false;
			} else {
				// IS THIS THE FIRST TIME IN LOOP?
				if (driveMode == false) {
					gyro.reset();
				}
				gyroOffset = (getGyro() * Preferences.getInstance().getDouble("GyroStrafeConstant", .01111111));
				driveMode = true;
				// USE THE GYRO FOR ROTATION ASSISTANCE
				if (Math.abs(gyroOffset) > 1) {
					gyroOffset = (Math.abs(gyroOffset) / gyroOffset);
				}
				robotDrive.mecanumDrive_Cartesian(stick.getX() * joystickScalerX, stick.getY() * joystickScalerY,
						-gyroOffset, 0);
			}
			SmartDashboard.putBoolean("DriveMode", driveMode);
		} else {
			robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
			gyro.reset();
		}
	}

	// STRAFE USING GYRO ASSISTANCE
	public void strafeWithGyro(int direction) { // -1 is left, 1 is right
		// SETTING THE GYRO STRAFE CONSTANT
		gyroOffset = (getGyro() * Preferences.getInstance().getDouble("GyroStrafeConstant", .01111111111));
		// IF THE ABOSOLUTE VAL OF THE GYRO OFFSET IS LARGER THAN 1
		if (Math.abs(gyroOffset) > 1) {
			// SET THE GYRO OFFSET TO EITHER 1 OR -1
			gyroOffset = (Math.abs(gyroOffset) / gyroOffset);
		}
		// STRAFE AT SOME POWER WHILE USING THE GYRO TO CORRECT FOR ROTATION
		robotDrive.mecanumDrive_Cartesian(Preferences.getInstance().getDouble("StrafePower", 0) * direction, 0,
				-gyroOffset, 0);
	}

	// DRIVES FORWARD WITH USING GYRO ASSISTANCE
	public void driveForwardWithGyro() {
		gyroOffset = (getGyro() * Preferences.getInstance().getDouble("GyroStrafeConstant", .01111111111));
		// IF THE ABOSOLUTE VAL OF THE GYRO OFFSET IS LARGER THAN 1
		if (Math.abs(gyroOffset) > 1) {
			// SET THE GYRO OFFSET TO EITHER 1 OR -1
			gyroOffset = (Math.abs(gyroOffset) / gyroOffset);
		}
		// STRAFE AT SOME POWER WHILE USING THE GYRO TO CORRECT FOR ROTATION
		robotDrive.mecanumDrive_Cartesian(0, (Preferences.getInstance().getDouble("autoSpeed", 0)), -gyroOffset, 0);
	}

	// MOVES THE ROBOT AT A GIVEN SPEED AT A GIVEN ANGLE
	public void driveAngle(int angle) {
		robotDrive.mecanumDrive_Polar(Preferences.getInstance().getDouble("OldManSpeed", 0), angle, 0);
	}

	// CENTERS THE ROBOT ON THE STEP
	public void centerOnStep() {
		gyroOffset = (getGyro() * Preferences.getInstance().getDouble("GyroStrafeConstant", .01111111111));
		if (Math.abs(gyroOffset) > 1) {
			gyroOffset = (Math.abs(gyroOffset) / gyroOffset);
		}
		centerOffsetDistance = getRightDistance() - getLeftDistance();
		if (isUltrasonicDataGood()) {
			robotDrive.mecanumDrive_Cartesian(
					centerOffsetDistance / Preferences.getInstance().getDouble("USCenterDistanceConstant", 27.5), 0,
					-gyroOffset, 0);
		} else {
			stopMotors();
		}
	}

	// CHECKS IF THE ROBOT IS CENTERED ON THE STEP
	public boolean isCentered() {
		return (Math.abs(getRightDistance() - getLeftDistance()) < 1);
	}

	// STOPS ALL OF THE MOTORS
	public void stopMotors() {
		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
	}

	// RETRIEVES THE DISTANCE THAT THE FRONT ULTRASONIC SENSOR IS FROM AN OBJECT
	public double getFrontDistance() {
		frontUSDistance = (frontRangefinder.getAverageVoltage() * USConstant);
		SmartDashboard.putNumber("Front Distance", frontUSDistance);
		return frontUSDistance;
	}

	// RETRIEVES THE DISTANCE THAT THE RIGHT ULTRASONIC SENSOR IS FROM AN OBJECT
	public double getRightDistance() {
		rightUSDistance = (rightRangefinder.getAverageVoltage() * USConstant);
		SmartDashboard.putNumber("Right Distance", rightUSDistance);
		return rightUSDistance;
	}

	// RETRIEVES THE DISTANCE THAT THE LEFT ULTRASONIC SENSOR IS FROM AN OBJECT
	public double getLeftDistance() {
		leftUSDistance = (leftRangefinder.getAverageVoltage() * USConstant);
		SmartDashboard.putNumber("Left Distance", leftUSDistance);
		return leftUSDistance;
	}

	// CHECKS IF THE ULRASONIC DATA IS REASONABLE
	public boolean isUltrasonicDataGood() {
		return (getLeftDistance() + getRightDistance() < 30);
	}

	// SETS UP THE GYRO
	public void initGyro() {
		gyro.initGyro();
		gyro.setSensitivity(.0125);

	}

	// RETURNS THE ANGLE AND RATE OF THE GYRO
	public double getGyro() {
		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		SmartDashboard.putNumber("Gyro Rate", gyro.getRate());
		return gyro.getAngle();
	}

	// RETURNS THE CURRENT OF THE MOTOR ON PORT 0
	public double getMotorCurrent() {
		return pdp.getCurrent(0);
	}

	// PRINTS THE CURRENT OF THE MOTOR ON PORT 0 TO THE SMARTDASHBOARD
	public void printPDPAverageCurrent() {
		SmartDashboard.putNumber("Average Current", pdp.getCurrent(0));
	}

	// PRINTS THE ENCODER SPEEDS
	public void printEncoderValues() {
		double inchesTraveled = (frontLeftEncoder.getDistance() / 1024)
				* (2 * Preferences.getInstance().getDouble("WheelRadius", 3) * Math.PI);
		SmartDashboard.putNumber("Inches Traveled", inchesTraveled);
		SmartDashboard.putNumber("Encoder Speed FL", frontLeftEncoder.getRate());
		SmartDashboard.putNumber("Encoder Speed FR", frontRightEncoder.getRate());
		SmartDashboard.putNumber("Encoder Speed RR", rearRightEncoder.getRate());
		SmartDashboard.putNumber("Encoder Speed RL", rearLeftEncoder.getRate());
	}

	// SETS THE PID CONSTANTS THROUGH PREFERENCES (CURRENTLY UNUSED)
	public void setPIDConstants() {
		double p = Preferences.getInstance().getDouble("DriveP", 0);
		double i = Preferences.getInstance().getDouble("DriveI", 0);
		double d = Preferences.getInstance().getDouble("DriveD", 0);
		double f = Preferences.getInstance().getDouble("DriveF", 0);

		frontLeftPID.setConstants(p, i, d, f);
		frontRightPID.setConstants(p, i, d, f);
		rearLeftPID.setConstants(p, i, d, f);
		rearRightPID.setConstants(p, i, d, f);
	}

	// RESETS THE INTEGRAL BUILDUP
	public void resetIntegral() {
		frontLeftPID.reset();
		frontRightPID.reset();
		rearLeftPID.reset();
		rearRightPID.reset();
	}

	@Override
	// DEFAULT COMMAND
	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoysticks());
	}
}