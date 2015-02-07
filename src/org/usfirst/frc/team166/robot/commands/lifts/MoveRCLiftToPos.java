package org.usfirst.frc.team166.robot.commands.lifts;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team166.robot.Robot;

/**
 *
 */
public class MoveRCLiftToPos extends Command {

	private double liftPosition;

	public MoveRCLiftToPos(double position) {
		requires(Robot.rcLift);
		liftPosition = position;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.rcLift.moveLiftToPosition(liftPosition);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return Robot.rcLift.isAtTargetPos(liftPosition);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.rcLift.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
