package org.usfirst.frc.team166.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.subsystems.Drive.ForwardBackwardDirection;

/**
 *
 */
public class DriveForwardBackwardDistance extends Command {
	private double driveSpeed;
	private ForwardBackwardDirection direction;
	private int distance;

	public DriveForwardBackwardDistance(double speed, ForwardBackwardDirection desiredDirection, int desiredDistance) {
		requires(Robot.drive);
		speed = driveSpeed;
		desiredDirection = direction;
		desiredDistance = distance;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.drive.driveForwardBackwardDistance(driveSpeed, direction, distance);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
