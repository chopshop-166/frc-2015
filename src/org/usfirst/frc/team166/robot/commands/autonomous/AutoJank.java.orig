package org.usfirst.frc.team166.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.commands.claw.ToggleClaw;
import org.usfirst.frc.team166.robot.commands.jankshank.ToggleShank;

/**
 *
 */
public class AutoJank extends CommandGroup {

	public AutoJank() {

		addSequential(new ToggleShank());
		addSequential(new DriveForwardBackwardDistance(0, 0, 50), 0.85);
		addSequential(new DriveForwardBackwardDistance(.75, 0, 70));
		addSequential(new ToggleShank());
		addSequential(new ToggleClaw());
		// addSequential(new TurnLeftToAngle(90));
	}
}
