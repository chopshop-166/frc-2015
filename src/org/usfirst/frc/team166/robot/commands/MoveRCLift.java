package org.usfirst.frc.team166.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team166.robot.Robot;

/**
 *
 */
public class MoveRCLift extends Command {

	public MoveRCLift() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.rcLift);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.oi.getRCJoystick();
		if (Robot.rcLift.isCarriageLimitHit() == true) // if the carriage for the tote hits the RC lift
		{
			cancel();// this goes to interrupted, I think

		}
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
