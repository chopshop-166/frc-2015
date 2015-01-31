package org.usfirst.frc.team166.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LiftWings extends CommandGroup {

	public LiftWings() {
		addParallel(new RaiseRightWing()); // raises both wings at the same time.
		addSequential(new RaiseLeftWing());
	}
}
