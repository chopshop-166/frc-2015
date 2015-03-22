package org.usfirst.frc.team166.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team166.robot.Robot;

/**
 *
 */
public class ToggleDriveSlowSpeed extends Command {

	public ToggleDriveSlowSpeed() {
		// Does not require a subsystem because the command only changes a software value
		// and does not actuate anything on the robot
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		if (Robot.drive.DriveSpeedModifier == 1) {
			Robot.drive.DriveSpeedModifier = .5;
		} else {
			Robot.drive.DriveSpeedModifier = 1;
		}

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return true;
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
