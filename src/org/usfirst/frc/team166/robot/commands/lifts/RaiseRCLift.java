package org.usfirst.frc.team166.robot.commands.lifts;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team166.robot.Robot;

/**
 *
 */
public class RaiseRCLift extends Command {

	public RaiseRCLift() {
		requires(Robot.rcLift);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.rcLift.moveUp();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.rcLift.updatePIDSetpoint(-1);
		Robot.rcLift.printEncoderValues();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return Robot.rcLift.isBoundaryHit();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.rcLift.stop();
		Robot.rcLift.resetEncoder();
		Robot.rcLift.updatePIDSetpoint(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.rcLift.stop();
		Robot.rcLift.resetEncoder();
	}
}
