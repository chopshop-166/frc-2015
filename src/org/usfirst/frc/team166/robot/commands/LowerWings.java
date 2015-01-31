package org.usfirst.frc.team166.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.Robot;

/**
 * Command group for lowering wings
 */
public class LowerWings extends CommandGroup {

	public LowerWings() {
		// wings to be lowered
		requires(Robot.leftWing);
		requires(Robot.rightWing);
		// starts lowering both wings at the same time
		addParallel(new LowerLeftWing());
		addSequential(new LowerRightWing());
	}
}
