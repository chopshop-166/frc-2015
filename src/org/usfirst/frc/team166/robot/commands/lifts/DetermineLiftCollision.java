package org.usfirst.frc.team166.robot.commands.lifts;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
			new ToteForcedDown().start();
			SmartDashboard.putString("Moving lift: ", "RC");
			break;
		case Tote:
			new RCForcedUp().start();
			SmartDashboard.putString("Moving lift: ", "Tote");
			break;
		case Both:
			Robot.rcLift.stop();
			Robot.toteLift.stop();
			SmartDashboard.putString("Moving lift: ", "Both");
			break;
		case None:
			// Deliberately empty, barren, desolate, devoid of activity, deadsies
			SmartDashboard.putString("Moving lift: ", "None");
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
