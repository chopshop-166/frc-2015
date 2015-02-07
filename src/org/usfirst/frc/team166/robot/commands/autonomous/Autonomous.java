package org.usfirst.frc.team166.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.commands.lifts.InitLiftEncoders;
import org.usfirst.frc.team166.robot.commands.wings.RaiseLeftWing;
import org.usfirst.frc.team166.robot.commands.wings.RaiseRightWing;

/**
 *
 */
public class Autonomous extends CommandGroup {

	public Autonomous() {
		addParallel(new InitLiftEncoders());
		addSequential(new DriveToStep());
		addSequential(new CenterOnStep(), 3.0);
		addSequential(new DriveDirection(0), .5);
		addSequential(new RaiseLeftWing());
		addParallel(new RaiseRightWing());
	}
}
