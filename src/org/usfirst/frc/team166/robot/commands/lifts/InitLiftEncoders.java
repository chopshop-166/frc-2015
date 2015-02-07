package org.usfirst.frc.team166.robot.commands.lifts;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class InitLiftEncoders extends CommandGroup {

	public InitLiftEncoders() {

		addSequential(new LowerToteLift());
		addSequential(new LowerRCLift());
		addSequential(new ResetToteLiftEncoder());
		addSequential(new ResetRCLiftEncoder());

	}
}
