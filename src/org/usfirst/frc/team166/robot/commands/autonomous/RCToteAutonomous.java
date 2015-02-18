package org.usfirst.frc.team166.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.commands.claw.CloseClaw;
import org.usfirst.frc.team166.robot.commands.lifts.LowerToteLift;
import org.usfirst.frc.team166.robot.commands.lifts.RaiseRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.RaiseToteLift;

/**
 *
 */
public class RCToteAutonomous extends CommandGroup {

	public RCToteAutonomous() {
		addParallel(new LowerToteLift());
		addSequential(new CloseClaw());
		addSequential(new RaiseRCLift(), 3);
		addParallel(new RaiseRCLift());
		addSequential(new DriveForwardBackwardDistance(.2, 0, 20));
		addParallel(new RaiseToteLift(), 3);
		addSequential(new TurnLeftToAngle());
		addSequential(new DriveForwardBackwardDistance(.25, 0, 30));

	}
}
