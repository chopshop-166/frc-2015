package org.usfirst.frc.team166.robot.commands.lifts;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.subsystems.Lift.LiftDirection;

//I JUST RENAMED POSITION TO DISTANCE, SINCE WE ARE WORKING RELATIVELY. Changed the name of  your variable in this command
//and the name of danny's function in lift

public class MoveToteLiftDistance extends Command {
	private double desiredDistance;
	private LiftDirection desiredDirection;

	public MoveToteLiftDistance(LiftDirection direction, double moveDistance) {
		requires(Robot.toteLift);
		desiredDirection = direction;
		desiredDistance = moveDistance;
		// if (direction == LiftDirection.up) {
		// desiredDistance = Math.abs(moveDistance);
		// }
		// if (direction == LiftDirection.down) {
		// desiredDistance = -Math.abs(moveDistance);
		// }
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.toteLift.resetEncoder();
		if (desiredDirection == LiftDirection.up) {
			SmartDashboard.putNumber("Command Tote Count", Robot.toteCount);
			// desiredDistance += (.035 * Robot.toteCount);
			Robot.toteCount += 1;
			Robot.toteLift.moveUp();
		}
		if (desiredDirection == LiftDirection.down) {
			Robot.toteCount = 0;
			Robot.toteLift.moveDown();
		}
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		SmartDashboard.putNumber("Current Tote Encoder Distance:", Robot.toteLift.getEncoderDistance());
		if (desiredDirection == LiftDirection.up) {
			SmartDashboard.putString("Tote Lift State:", "Moving up");
			Robot.toteLift.updatePIDSetpoint(1);
		}
		if (desiredDirection == LiftDirection.down) {
			Robot.toteLift.updatePIDSetpoint(-1);
			SmartDashboard.putString("Tote Lift State:", "Moving down");
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (desiredDirection == LiftDirection.up) {
			return (Math.abs(Robot.toteLift.getEncoderDistance()) >= desiredDistance)
					|| (Robot.rcLift.areLiftsInContact() && Robot.rcLift.isBoundaryHit() || (Robot.rcLift
							.areLiftsInContact() && Robot.toteLift.isBoundaryHit()));
		} else {
			return (Math.abs(Robot.toteLift.getEncoderDistance()) >= desiredDistance) || Robot.toteLift.isBoundaryHit();
		}
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		SmartDashboard.putString("Tote Lift State:", "Not Moving");
		Robot.toteLift.stop();
		Robot.toteLift.resetEncoder();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
