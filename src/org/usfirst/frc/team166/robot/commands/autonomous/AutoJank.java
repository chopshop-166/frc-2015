package org.usfirst.frc.team166.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.commands.claw.ToggleClaw;
import org.usfirst.frc.team166.robot.commands.claw.ToggleJank;

/**
 *
 */
public class AutoJank extends CommandGroup {

	public AutoJank() {

		addSequential(new ToggleJank());
		addSequential(new DriveForwardBackwardDistance(0, 0, 50), 0.85);
		addSequential(new DriveForwardBackwardDistance(.5, 0, 70));
		addSequential(new ToggleJank());
		addSequential(new ToggleClaw());
		// addSequential(new TurnLeftToAngle(90));
	}
}
