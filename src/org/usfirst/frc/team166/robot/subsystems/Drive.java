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
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team166.robot.PIDSpeedController;
import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.RobotMap;
import org.usfirst.frc.team166.robot.Utility;
import org.usfirst.frc.team166.robot.commands.drive.DriveWithJoysticks;

public class Drive extends Subsystem {

	// Constants
	private static final double GyroSensitivity = .0125; // V/Deg/Sec
	private static final double UltrasonicConstant = 102.0408163265306; // inch/V
	private static final double DistanceNormal = 91.5;
	private static final double DistancePerPulse = ((6 * Math.PI) / 1024) / DistanceNormal;

	// Slow Speed Global Var
	public double DriveSpeedModifier = 1.0;
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

	boolean usingTwist = false;

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
		frontLeftEncoder.setDistancePerPulse(DistancePerPulse);
		frontLeftEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);
		frontRightEncoder.setDistancePerPulse(DistancePerPulse);
		frontRightEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);
		rearLeftEncoder.setDistancePerPulse(DistancePerPulse);
		rearLeftEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);
		rearRightEncoder.setDistancePerPulse(DistancePerPulse);
		rearRightEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);

		// PID SPEED CONTROLLERS
		frontLeftPID = new PIDSpeedController(frontLeftEncoder, frontLeftTalon, "Drive", "Front Left");
		frontRightPID = new PIDSpeedController(frontRightEncoder, frontRightTalon, "Drive", "Front Right");
		rearLeftPID = new PIDSpeedController(rearLeftEncoder, rearLeftTalon, "Drive", "Rear Left");
		rearRightPID = new PIDSpeedController(rearRightEncoder, rearRightTalon, "Drive", "Rear Right");

		// DRIVE DECLARATION
		robotDrive = new RobotDrive(frontLeftPID, rearLeftPID, frontRightPID, rearRightPID);
		robotDrive.setExpiration(0.1);

		// MOTOR INVERSIONS
		robotDrive.setInvertedMotor(MotorType.kFrontRight, true);
		robotDrive.setInvertedMotor(MotorType.kRearRight, true);

		LiveWindow.addActuator("Drive", "Front Left Talon", frontLeftTalon);
		LiveWindow.addActuator("Drive", "Front Right Talon", frontRightTalon);
		LiveWindow.addActuator("Drive", "Rear Left Talon", rearLeftTalon);
		LiveWindow.addActuator("Drive", "Rear Right Talon", rearRightTalon);

		LiveWindow.addSensor("Drive Encoders", "Front Left Encoder", frontLeftEncoder);
		LiveWindow.addSensor("Drive Encoders", "Front Right Encoder", frontRightEncoder);
		LiveWindow.addSensor("Drive Encoders", "Rear Left Encoder", rearLeftEncoder);
		LiveWindow.addSensor("Drive Encoders", "Rear Right Encoder", rearRightEncoder);
	}

	// MECANUMDRIVE USING DEADZONES, GYRO, AND ENCODERS
	public void mecanumDrive(Joystick stick) {

		// DRIVE FORWARD WITH JOYSTICKS ONLY
		if (!Utility.isAxisZero(Robot.oi.getDriveJoystickRotation())) {
			robotDrive.mecanumDrive_Cartesian(Robot.oi.getDriveJoystickLateral(), Robot.oi.getDriveJoystickForward(),
					Robot.oi.getDriveJoystickRotation(), 0);
			usingTwist = true;
		} else if ((!Utility.isAxisZero(Robot.oi.getDriveJoystickLateral()))
				|| !Utility.isAxisZero((Robot.oi.getDriveJoystickForward()))) {
			// IS THIS THE FIRST TIME IN LOOP?
			if (usingTwist == true) {
				gyro.reset();
			}
			usingTwist = false;
			// USE THE GYRO FOR ROTATION ASSISTANCE
			robotDrive.mecanumDrive_Cartesian(Robot.oi.getDriveJoystickLateral(), Robot.oi.getDriveJoystickForward(),
					getGyroOffset(), 0);
		} else {
			robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
			gyro.reset();
		}
	}

	// DRIVES FORWARD WITH USING GYRO ASSISTANCE
	public void driveForwardWithGyro() {
		robotDrive.mecanumDrive_Cartesian(0, (Preferences.getInstance().getDouble(RobotMap.Prefs.AutoDriveSpeed, 0)),
				getGyroOffset(), 0);
	}

	private double getGyroOffset() {
		double gyroOffset = (getGyro() * Preferences.getInstance().getDouble(RobotMap.Prefs.GyroStrafeConstant,
				.01111111111));
		// IF THE ABOSOLUTE VAL OF THE GYRO OFFSET IS LARGER THAN 1
		if (Math.abs(gyroOffset) > 1) {
			// SET THE GYRO OFFSET TO EITHER 1 OR -1
			return (-1 * (Math.abs(gyroOffset) / gyroOffset));
		} else {
			return -gyroOffset;
		}
	}

	public void resetGyro() {
		gyro.reset();
	}

	// MOVES THE ROBOT AT A GIVEN SPEED AT A GIVEN ANGLE
	public void driveAngle(double angle, double speed) {
		robotDrive.mecanumDrive_Polar(speed, angle, getGyroOffset()); // You're dumb
		// robotDrive.mecanumDrive_Cartesian(.5, 0, getGyroOffset(), 0);
	}

	// CENTERS THE ROBOT ON THE STEP
	public void centerOnStep() {
		double centerOffsetDistance = getRightDistance() - getLeftDistance();
		if (isUltrasonicDataGood()) {
			robotDrive.mecanumDrive_Cartesian(
					centerOffsetDistance
							/ Preferences.getInstance().getDouble(RobotMap.Prefs.CenterDistanceConstant, 27.5), 0,
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
		double frontUSDistance = (frontRangefinder.getAverageVoltage() * UltrasonicConstant);
		SmartDashboard.putNumber("Front Distance", frontUSDistance);
		return frontUSDistance;
	}

	// RETRIEVES THE DISTANCE THAT THE RIGHT ULTRASONIC SENSOR IS FROM AN OBJECT
	public double getRightDistance() {
		double rightUSDistance = (rightRangefinder.getAverageVoltage() * UltrasonicConstant);
		SmartDashboard.putNumber("Right Distance", rightUSDistance);
		return rightUSDistance;
	}

	// RETRIEVES THE DISTANCE THAT THE LEFT ULTRASONIC SENSOR IS FROM AN OBJECT
	public double getLeftDistance() {
		double leftUSDistance = (leftRangefinder.getAverageVoltage() * UltrasonicConstant);
		SmartDashboard.putNumber("Left Distance", leftUSDistance);
		return leftUSDistance;
	}

	// RETURNS WHETHER OR NOT THE DRIVE MOTORS ARE STALLED
	public boolean isStalled() {
		return (Robot.pdBoard.getCurrent(RobotMap.Pwm.FrontLeftDrive) > Preferences.getInstance().getDouble(
				RobotMap.Prefs.StalledDriveCurrent, 20)
				|| Robot.pdBoard.getCurrent(RobotMap.Pwm.FrontRightDrive) > Preferences.getInstance().getDouble(
						RobotMap.Prefs.StalledDriveCurrent, 20)
				|| Robot.pdBoard.getCurrent(RobotMap.Pwm.RearLeftDrive) > Preferences.getInstance().getDouble(
						RobotMap.Prefs.StalledDriveCurrent, 20) || Robot.pdBoard
				.getCurrent(RobotMap.Pwm.RearRightDrive) > Preferences.getInstance().getDouble(
				RobotMap.Prefs.StalledDriveCurrent, 20));
	}

	// CHECKS IF THE ULRASONIC DATA IS REASONABLE
	public boolean isUltrasonicDataGood() {
		// 30 IS THE TOTAL DISTANCE OF THE GAP IN AUTONOMOUS - THE WIDTH OF THE CHASIS
		return (getLeftDistance() + getRightDistance() < 30);
	}

	// SETS UP THE GYRO
	public void initGyro() {
		gyro.initGyro();
		gyro.setSensitivity(GyroSensitivity);
	}

	// RETURNS THE ANGLE AND RATE OF THE GYRO
	public double getGyro() {
		double angle = gyro.getAngle();
		SmartDashboard.putNumber("Gyro Angle", angle);
		SmartDashboard.putNumber("Gyro Rate", gyro.getRate());
		return angle;
	}

	// PRINTS THE ENCODER SPEEDS
	public void printEncoderValues() {
		SmartDashboard.putNumber("Encoder Speed FL", frontLeftEncoder.getRate());
		SmartDashboard.putNumber("Encoder Speed FR", frontRightEncoder.getRate());
		SmartDashboard.putNumber("Encoder Speed RR", rearRightEncoder.getRate());
		SmartDashboard.putNumber("Encoder Speed RL", rearLeftEncoder.getRate());
	}

	public double getDistanceTraveled(Encoder driveEncoder) {
		double distanceTraveled = (driveEncoder.getDistance() * DistanceNormal);
		return distanceTraveled;
	}

	public enum ForwardBackwardDirection {
		Forward, Backward
	}

	// andrew is dumb

	// SETS THE PID CONSTANTS THROUGH PREFERENCES (CURRENTLY UNUSED)
	public void setPIDConstants() {
		double p = Preferences.getInstance().getDouble(RobotMap.Prefs.DriveSpeedP, 0);
		double i = Preferences.getInstance().getDouble(RobotMap.Prefs.DriveSpeedI, 0);
		double d = Preferences.getInstance().getDouble(RobotMap.Prefs.DriveSpeedD, 0);
		double f = Preferences.getInstance().getDouble(RobotMap.Prefs.DriveSpeedF, 0);

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

	public double getEncoderDistance() {
		double distanceTraveledAverage = (getDistanceTraveled(frontRightEncoder)
				+ getDistanceTraveled(frontLeftEncoder) + getDistanceTraveled(rearRightEncoder) + getDistanceTraveled(rearLeftEncoder))
				/ 4 * DistanceNormal;
		return distanceTraveledAverage;
	}

	public void resetEncoders() {
		frontRightEncoder.reset();
		frontLeftEncoder.reset();
		rearRightEncoder.reset();
		rearLeftEncoder.reset();
	}

	@Override
	// DEFAULT COMMAND
	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoysticks());
	}
}