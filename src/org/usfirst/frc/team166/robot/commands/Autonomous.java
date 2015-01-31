package org.usfirst.frc.team166.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Autonomous extends CommandGroup {

	public Autonomous() {
		addSequential(new DriveToStep());
		addSequential(new CenterOnStep(), 3.0);
		addSequential(new DriveDirection(180), .5);
		addSequential(new RaiseLeftWing());
		addParallel(new RaiseRightWing());
	}
}
