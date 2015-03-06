package org.usfirst.frc.team166.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team166.robot.Robot;

/**
 *
 */
public class DriveForwardBackwardDistance extends Command {
	private double driveSpeed;
	private double direction;
	private int distance;

	public DriveForwardBackwardDistance(double speed, double angle, int desiredDistance) {
		requires(Robot.drive);
		driveSpeed = speed;
		direction = angle;
		distance = desiredDistance;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.drive.resetGyro();
		Robot.drive.resetEncoders();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Robot.drive.driveForwardBackwardDistance(driveSpeed, direction, distance);
		SmartDashboard.putNumber("Distance Traveled Command", Robot.drive.getEncoderDistance());
		Robot.drive.driveAngle(direction, driveSpeed);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return (Math.abs(Robot.drive.getEncoderDistance()) > distance);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.drive.stopMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
