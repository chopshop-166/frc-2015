package org.usfirst.frc.team166.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.commands.lifts.LowerRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.LowerToteLift;
import org.usfirst.frc.team166.robot.commands.lifts.ResetRCLiftEncoder;
import org.usfirst.frc.team166.robot.commands.lifts.ResetToteLiftEncoder;

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
