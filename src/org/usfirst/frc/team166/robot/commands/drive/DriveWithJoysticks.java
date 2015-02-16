package org.usfirst.frc.team166.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team166.robot.Robot;

/**
 *
 */
public class DriveWithJoysticks extends Command {

	public DriveWithJoysticks() {
		requires(Robot.drive);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.drive.setPIDConstants();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.drive.mecanumDrive(Robot.oi.getDriveJoystick());
		Robot.drive.printEncoderValues();
		Robot.drive.getGyro();
		Robot.drive.getFrontDistance();
		Robot.drive.getRightDistance();
		Robot.drive.getLeftDistance();
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
