package org.usfirst.frc.team166.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.Robot;

/**
 *
 */
public class LiftWings extends CommandGroup {

	public LiftWings() {
		requires(Robot.rightWing);
		requires(Robot.leftWing);
		addParallel(new RaiseRightWing()); // raises both wings at the same time.
		addSequential(new RaiseLeftWing());
	}
}
