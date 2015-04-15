package org.usfirst.frc.team166.robot.commands.lifts;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.subsystems.Lift.LiftDirection;

/**
 *
 */
public class MoveToteLiftDistance extends Command {
	private double desiredPosition;
	private LiftDirection desiredDirection;

	public MoveToteLiftDistance(LiftDirection direction, int moveDistance) {
		requires(Robot.toteLift);
		desiredDirection = direction;
		if (direction == LiftDirection.up) {
			desiredPosition = Math.abs(moveDistance);
		}
		if (direction == LiftDirection.down) {
			desiredPosition = -Math.abs(moveDistance);
		}
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.toteLift.resetEncoder();
		if (desiredDirection == LiftDirection.up) {
			Robot.toteLift.moveUp();
		}
		if (desiredDirection == LiftDirection.down) {
			Robot.toteLift.moveDown();
		}
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (desiredDirection == LiftDirection.up) {
			Robot.rcLift.updatePIDSetpoint(-1);
		}
		if (desiredDirection == LiftDirection.down) {
			Robot.rcLift.updatePIDSetpoint(1);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return Robot.toteLift.isAtTargetPos(desiredPosition)
				|| Robot.toteLift.isBoundaryHit()
				|| (Robot.rcLift.areLiftsInContact() && Robot.rcLift.isBoundaryHit() || (Robot.rcLift
						.areLiftsInContact() && Robot.toteLift.isBoundaryHit()));
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
