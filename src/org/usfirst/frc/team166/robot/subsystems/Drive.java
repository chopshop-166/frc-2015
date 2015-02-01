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
import org.usfirst.frc.team166.robot.commands.DriveWithJoysticks;

public class Drive extends Subsystem {
	// ROBOTDRIVE INIT
	RobotDrive robotDrive;

	// PDP INIT
	PowerDistributionPanel pdp;

	// RANGEFINDER INITS
	AnalogInput frontRangefinder;
	AnalogInput leftRangefinder;
	AnalogInput rightRangefinder;

	// PID CONTROLLER INITS
	PIDSpeedController frontLeftPID;
	PIDSpeedController frontRightPID;
	PIDSpeedController rearLeftPID;
	PIDSpeedController rearRightPID;

	// TALON INITS
	Talon frontLeftTalon;
	Talon rearLeftTalon;
	Talon frontRightTalon;
	Talon rearRightTalon;

	// ENCODER INITS
	Encoder frontLeftEncoder;
	Encoder frontRightEncoder;
	Encoder rearLeftEncoder;
	Encoder rearRightEncoder;

	// YAWRATE SENSOR INIT
	public Gyro gyro;

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
		rearLeftTalon = new Talon(0);
		rearRightTalon = new Talon(1);
		frontLeftTalon = new Talon(2);
		frontRightTalon = new Talon(3);

		// ULTRASONIC SENSORS
		frontRangefinder = new AnalogInput(1);
		rightRangefinder = new AnalogInput(2);
		leftRangefinder = new AnalogInput(3);

		// YAWRATE SENSOR
		gyro = new Gyro(0);

		// ENCODER PORTS
		rearLeftEncoder = new Encoder(0, 1);
		rearRightEncoder = new Encoder(2, 3);
		frontLeftEncoder = new Encoder(4, 5);
		frontRightEncoder = new Encoder(6, 7);

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
		frontLeftPID = new PIDSpeedController(frontLeftEncoder, frontLeftTalon, "frontLeft");
		frontRightPID = new PIDSpeedController(frontRightEncoder, frontRightTalon, "frontRight");
		rearLeftPID = new PIDSpeedController(rearLeftEncoder, rearLeftTalon, "rearLeft");
		rearRightPID = new PIDSpeedController(rearRightEncoder, rearRightTalon, "rearRight");

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
	public void strafeWithGyro(StrafeDirection direction) {
		int multiplier = (direction == StrafeDirection.Left) ? -1 : 1;
		// STRAFE AT SOME POWER WHILE USING THE GYRO TO CORRECT FOR ROTATION
		robotDrive.mecanumDrive_Cartesian(Preferences.getInstance().getDouble("StrafePower", .25) * multiplier, 0,
				getGyroOffset(), 0);
	}

	// DRIVES FORWARD WITH USING GYRO ASSISTANCE
	public void driveForwardWithGyro() {
		robotDrive.mecanumDrive_Cartesian(0, (Preferences.getInstance().getDouble("autoSpeed", 0)), getGyroOffset(), 0);
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
	public void driveAngle(double angle) {
		robotDrive
		.mecanumDrive_Polar(Preferences.getInstance().getDouble("DriveAngleSpeed", 0), angle, getGyroOffset());
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
		frontLeftPID.setConstants();
		frontRightPID.setConstants();
		rearLeftPID.setConstants();
		rearRightPID.setConstants();
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