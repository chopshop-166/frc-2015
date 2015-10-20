package org.usfirst.frc.team166.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.RobotMap;
import org.usfirst.frc.team166.robot.commands.drive.DriveDirection;

/**
 *
 */
public class FourStackAutonomous extends CommandGroup {
	double autoForwardSpeed = Preferences.getInstance().getDouble(RobotMap.Prefs.AutoForwardSpeed, 0.3);

	public FourStackAutonomous() {
		addSequential(new StackTwoTotes());
		addSequential(new DriveForwardBackwardDistance(.6, 180, 12));
		addSequential(new DriveDirection(-90, .6), 1.5);
		addSequential(new StackTwoTotes());
		addSequential(new DriveForwardBackwardDistance(.5, 180, 50));
	}
}
