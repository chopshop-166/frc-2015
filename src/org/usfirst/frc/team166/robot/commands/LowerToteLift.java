package org.usfirst.frc.team166.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team166.robot.Robot;

/**
 *
 */
public class LowerToteLift extends Command {

	public LowerToteLift() {
		requires(Robot.toteLift);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.toteLift.moveDown();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return Robot.toteLift.isBoundaryHit();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.toteLift.stop();
		Robot.toteLift.resetEncoder();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
