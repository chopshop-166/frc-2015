package org.usfirst.frc.team166.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.subsystems.Drive;

/**
 *
 */
public class StrafeDirection extends Command {
	private Drive.StrafeDirection strafeDirection;

	public StrafeDirection(Drive.StrafeDirection direction) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.drive);
		strafeDirection = direction;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.drive.strafeWithGyro(strafeDirection);
		Robot.drive.printEncoderValues();
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
