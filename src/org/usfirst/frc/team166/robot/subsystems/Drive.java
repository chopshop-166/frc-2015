package org.usfirst.frc.team166.robot.subsystems;

//IMPORTS
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team166.robot.PIDSpeedController;
import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.RobotMap;
import org.usfirst.frc.team166.robot.commands.drive.DriveWithJoysticks;

public class Drive extends Subsystem {
	// ROBOTDRIVE INIT
	final RobotDrive robotDrive;

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

		// SPEED CONTROLLER PORTS
		frontLeftTalon = new Talon(RobotMap.Pwm.FrontLeftDrive);
		rearLeftTalon = new Talon(RobotMap.Pwm.RearLeftDrive);
		frontRightTalon = new Talon(RobotMap.Pwm.FrontRightDrive);
		rearRightTalon = new Talon(RobotMap.Pwm.RearRightDrive);

		// ULTRASONIC SENSORS
		frontRangefinder = new AnalogInput(RobotMap.Analog.FrontRangeFinder);
		rightRangefinder = new AnalogInput(RobotMap.Analog.RightRangeFinder);
		leftRangefinder = new AnalogInput(RobotMap.Analog.LeftRangeFinder);

		// YAWRATE SENSOR
		gyro = new Gyro(RobotMap.Analog.Gryo);

		// ENCODER PORTS
		frontLeftEncoder = new Encoder(RobotMap.Encoders.FrontLeftDriveA, RobotMap.Encoders.FrontLeftDriveB);
		rearLeftEncoder = new Encoder(RobotMap.Encoders.RearLeftDriveA, RobotMap.Encoders.RearLeftDriveB);
		frontRightEncoder = new Encoder(RobotMap.Encoders.FrontRightDriveA, RobotMap.Encoders.FrontRightDriveB);
		rearRightEncoder = new Encoder(RobotMap.Encoders.RearRightDriveA, RobotMap.Encoders.RearRightDriveB);

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
		if ((Math.abs(stick.getX()) > Preferences.getInstance().getDouble("deadZone", .1))
				|| (Math.abs(stick.getY()) > Preferences.getInstance().getDouble("deadZone", .1))
				|| (Math.abs(stick.getRawAxis(3)) > .1)) {
			if (Math.abs(stick.getRawAxis(3)) > Preferences.getInstance().getDouble("deadZone", .1)) {
				robotDrive.mecanumDrive_Cartesian(stick.getX() * joystickScalerX, stick.getY() * joystickScalerY,
						stick.getRawAxis(3) * joystickScalerRotation, 0);
				driveMode = false;
			} else {
				// IS THIS THE FIRST TIME IN LOOP?
				if (driveMode == false) {
					gyro.reset();
				}
				driveMode = true;
				// USE THE GYRO FOR ROTATION ASSISTANCE
				robotDrive.mecanumDrive_Cartesian(stick.getX() * joystickScalerX, stick.getY() * joystickScalerY,
						getGyroOffset(), 0);
			}
			SmartDashboard.putBoolean("DriveMode", driveMode);
		} else {
			robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
			gyro.reset();
		}
	}

	public enum StrafeDirection {
		Left, Right
	}

	// STRAFE USING GYRO ASSISTANCE
	public void strafeWithGyro(StrafeDirection direction, double speed) {
		int multiplier = (direction == StrafeDirection.Left) ? -1 : 1;
		// STRAFE AT SOME POWER WHILE USING THE GYRO TO CORRECT FOR ROTATION
		robotDrive.mecanumDrive_Cartesian(speed * multiplier, 0, getGyroOffset(), 0);
	}

	// DRIVES FORWARD WITH USING GYRO ASSISTANCE
	public void driveForwardWithGyro() {
		robotDrive.mecanumDrive_Cartesian(0, (Preferences.getInstance().getDouble("AutoSpeed", 0)), getGyroOffset(), 0);
	}

	private double getGyroOffset() {
		double gyroOffset = (getGyro() * Preferences.getInstance().getDouble("GyroStrafeConstant", .01111111111));
		// IF THE ABOSOLUTE VAL OF THE GYRO OFFSET IS LARGER THAN 1
		if (Math.abs(gyroOffset) > 1) {
			// SET THE GYRO OFFSET TO EITHER 1 OR -1
			return (-1 * (Math.abs(gyroOffset) / gyroOffset));
		} else {
			return -gyroOffset;
		}
	}

	// MOVES THE ROBOT AT A GIVEN SPEED AT A GIVEN ANGLE
	public void driveAngle(double angle, double speed) {
		robotDrive.mecanumDrive_Polar(speed, angle, getGyroOffset());
	}

	// CENTERS THE ROBOT ON THE STEP
	public void centerOnStep() {
		centerOffsetDistance = getRightDistance() - getLeftDistance();
		if (isUltrasonicDataGood()) {
			robotDrive.mecanumDrive_Cartesian(
					centerOffsetDistance / Preferences.getInstance().getDouble("USCenterDistanceConstant", 27.5), 0,
					getGyroOffset(), 0);
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

	// RETURNS WHETHER OR NOT THE DRIVE MOTORS ARE STALLED
	public boolean isStalled() {
		return (Robot.pdBoard.getCurrent(RobotMap.Pwm.FrontLeftDrive) > Preferences.getInstance().getDouble(
				"currentCutoff", 20)
				|| Robot.pdBoard.getCurrent(RobotMap.Pwm.FrontRightDrive) > Preferences.getInstance().getDouble(
						"currentCutoff", 20)
				|| Robot.pdBoard.getCurrent(RobotMap.Pwm.RearLeftDrive) > Preferences.getInstance().getDouble(
						"currentCutoff", 20) || Robot.pdBoard.getCurrent(RobotMap.Pwm.RearRightDrive) > Preferences
				.getInstance().getDouble("currentCutoff", 20));
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

	// PRINTS THE CURRENT OF THE MOTOR ON PORT 0 TO THE SMARTDASHBOARD
	public void printPDPAverageCurrent() {
		SmartDashboard.putNumber("Average Current", Robot.pdBoard.getCurrent(RobotMap.Pwm.RearLeftDrive));
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