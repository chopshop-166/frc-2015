package org.usfirst.frc.team166.robot.commands.lifts;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team166.robot.subsystems.Lift;

/**
 *
 */
public class MoveLiftToPos extends Command {

	private double liftPosition;
	private Lift lift;

	public MoveLiftToPos(Lift m_lift, double position) {
		requires(m_lift);
		liftPosition = position;
		lift = m_lift;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		lift.moveLiftToPosition(liftPosition);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return true;
		// return lift.hasMovedDistance(liftPosition)
		// || lift.isBoundaryHit()
		// || (Robot.rcLift.areLiftsInContact() && Robot.rcLift.isBoundaryHit() || (Robot.rcLift
		// .areLiftsInContact() && Robot.toteLift.isBoundaryHit()));
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		lift.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
