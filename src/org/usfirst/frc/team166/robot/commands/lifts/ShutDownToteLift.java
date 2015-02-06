package org.usfirst.frc.team166.robot.commands.lifts;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class ShutDownToteLift extends CommandGroup {

	public ShutDownToteLift() {

		this.setInterruptible(false);

		addSequential(new StopRCLift());
		addSequential(new WaitCommand(1));
	}
}
