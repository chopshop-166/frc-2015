package org.usfirst.frc.team166.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team166.robot.Robot;

/**
 *
 */
public class DriveToStep extends Command {

	public DriveToStep() {
		requires(Robot.drive);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.drive.printPDPAverageCurrent();
		Robot.drive.driveForwardWithGyro();

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return (Robot.drive.getMotorCurrent() > Preferences.getInstance().getDouble("currentCutoff", 20));
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.drive.stopMotors();
		Robot.drive.resetIntegral();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
