package org.usfirst.frc.team166.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team166.robot.Robot;

/**
 *
 */
public class DriveToDistance extends Command {
	private double speed;

	public DriveToDistance(double driveSpeed) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drive);
		speed = driveSpeed;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.drive.resetGyro();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.drive.driveAngle(0, speed);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return (Robot.drive.distanceToTote() < 1);
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
