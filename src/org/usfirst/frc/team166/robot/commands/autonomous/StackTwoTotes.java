package org.usfirst.frc.team166.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.RobotMap;
import org.usfirst.frc.team166.robot.commands.drive.CenterOnTote;
import org.usfirst.frc.team166.robot.commands.lifts.LowerToteLift;
import org.usfirst.frc.team166.robot.commands.lifts.RaiseToteLift;

/**
 *
 */
public class StackTwoTotes extends CommandGroup {
	double autoForwardSpeed = Preferences.getInstance().getDouble(RobotMap.Prefs.AutoForwardSpeed, 0.3);

	public StackTwoTotes() {
		addParallel(new LowerToteLift());
		addSequential(new CenterOnTote(), 2.5);
		// addSequential(new DriveDirection(0, .1), .5);
		// addParallel(new DriveDirection(0, .1), .5);
		addSequential(new RaiseToteLift(), 1.65);
		addSequential(new DriveForwardBackwardDistance(.15, 0, 10)); // all of these were .1 before
		addSequential(new DriveToDrop());
		addSequential(new LowerToteLift(), .75);
		addParallel(new LowerToteLift());
		addSequential(new DriveForwardBackwardDistance(.11, 180, 5));
		addSequential(new CenterOnTote(), 2.5);
		// addSequential(new DriveDirection(0, .1), .5);
		// addParallel(new DriveDirection(0, .1), .5);
		addSequential(new RaiseToteLift(), 1);
		addSequential(new DriveForwardBackwardDistance(.11, 180, 10));
	}
}
