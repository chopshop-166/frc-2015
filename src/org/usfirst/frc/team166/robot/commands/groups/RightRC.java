package org.usfirst.frc.team166.robot.commands.groups;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.RobotMap;
import org.usfirst.frc.team166.robot.commands.claw.CloseClaw;
import org.usfirst.frc.team166.robot.commands.claw.OpenClaw;
import org.usfirst.frc.team166.robot.commands.drive.DriveDirection;
import org.usfirst.frc.team166.robot.commands.drive.StopDriveMotors;
import org.usfirst.frc.team166.robot.commands.lifts.MoveLiftToPos;

/**
 *
 */
public class RightRC extends CommandGroup {

	public RightRC() {
		addSequential(new OpenClaw());
		addSequential(new MoveLiftToPos(Robot.toteLift, Preferences.getInstance().getDouble(
				RobotMap.Prefs.RCRightToteLiftPos, 24)));
		addSequential(new MoveLiftToPos(Robot.rcLift, Preferences.getInstance().getDouble(
				RobotMap.Prefs.RCRightToteLiftPos, 30)));
		addParallel(new MoveLiftToPos(Robot.toteLift, Preferences.getInstance().getDouble(
				RobotMap.Prefs.RCRightToteLiftPos, 0)));
		addParallel(new DriveDirection(180, Preferences.getInstance().getDouble(RobotMap.Prefs.RightRCDirection, 0)));
		addSequential(new WaitCommand(Preferences.getInstance().getDouble(RobotMap.Prefs.RightRCWaitTime, 1)));
		addSequential(new StopDriveMotors());
		addSequential(new CloseClaw());
	}
}