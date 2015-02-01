package org.usfirst.frc.team166.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.StartCommand;

import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.subsystems.Lift;

/**
 *
 */
public class DetermineLiftCollision extends Command {

	Lift.WhichCarriagePushing movingCarriage;

	public DetermineLiftCollision() {
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		movingCarriage = Lift.collisionMovement(Robot.rcLift.getMoveState(), Robot.toteLift.getMoveState());
		switch (movingCarriage) {
		case RC:
			new StartCommand(new ToteForcedDown());
			break;
		case Tote:
			new StartCommand(new RCForcedUp());
			break;
		case Both:
			new StartCommand(new StopToteLift());
			new StartCommand(new StopRCLift());
			break;
		case None:
			// Deliberately empty, barren, desolate, devoid of activity, deadsies
			break;
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
