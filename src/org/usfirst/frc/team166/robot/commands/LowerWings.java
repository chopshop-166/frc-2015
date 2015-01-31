package org.usfirst.frc.team166.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Command group for lowering wings
 */
public class LowerWings extends CommandGroup {

	public LowerWings() {
		// command groups don't need explicit requires
		// starts lowering both wings at the same time
		addParallel(new LowerLeftWing());
		addSequential(new LowerRightWing());
	}
}
