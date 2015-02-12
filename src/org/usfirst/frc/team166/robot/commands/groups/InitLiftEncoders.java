package org.usfirst.frc.team166.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.commands.lifts.LowerLift;
import org.usfirst.frc.team166.robot.commands.lifts.ResetLiftEncoder;

/**
 *
 */
public class InitLiftEncoders extends CommandGroup {

	public InitLiftEncoders() {

		addSequential(new LowerLift(Robot.toteLift));
		addSequential(new LowerLift(Robot.rcLift));
		addSequential(new ResetLiftEncoder(Robot.toteLift));
		addSequential(new ResetLiftEncoder(Robot.rcLift));

	}
}
