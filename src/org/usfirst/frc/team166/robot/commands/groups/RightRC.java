package org.usfirst.frc.team166.robot.commands.groups;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

import org.usfirst.frc.team166.robot.RobotMap;
import org.usfirst.frc.team166.robot.commands.autonomous.DriveDirection;
import org.usfirst.frc.team166.robot.commands.claw.CloseClaw;
import org.usfirst.frc.team166.robot.commands.claw.OpenClaw;
import org.usfirst.frc.team166.robot.commands.drive.StopDriveMotors;
import org.usfirst.frc.team166.robot.commands.lifts.LowerRCLift;

/**
 *
 */
public class RightRC extends CommandGroup {

	public RightRC() {
		addSequential(new OpenClaw());
		addParallel(new LowerRCLift());
		addParallel(new DriveDirection(180, Preferences.getInstance().getDouble(RobotMap.Prefs.RightRCDirection, 0)));
		addSequential(new WaitCommand(Preferences.getInstance().getDouble(RobotMap.Prefs.RightRCWaitTime, 1)));
		addSequential(new StopDriveMotors());
		addSequential(new CloseClaw());
	}
}
