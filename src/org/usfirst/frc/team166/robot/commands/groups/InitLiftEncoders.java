package org.usfirst.frc.team166.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.commands.lifts.LowerRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.LowerToteLift;
import org.usfirst.frc.team166.robot.commands.lifts.ResetLiftEncoder;

/**
 *
 */
public class InitLiftEncoders extends CommandGroup {

	public InitLiftEncoders() {

		addSequential(new LowerToteLift());
		addSequential(new LowerRCLift());
		addSequential(new ResetLiftEncoder(Robot.toteLift));
		addSequential(new ResetLiftEncoder(Robot.rcLift));

	}
}
