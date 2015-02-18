package org.usfirst.frc.team166.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.commands.claw.CloseClaw;
import org.usfirst.frc.team166.robot.commands.claw.OpenClaw;
import org.usfirst.frc.team166.robot.commands.lifts.LowerToteLift;
import org.usfirst.frc.team166.robot.commands.lifts.RaiseRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.RaiseToteLift;

/**
 *
 */
public class RCToteAutonomous extends CommandGroup {

	public RCToteAutonomous() {
		addSequential(new OpenClaw());
		addParallel(new LowerToteLift());
		addSequential(new CloseClaw());
		addSequential(new RaiseRCLift(), 3);
		addParallel(new RaiseRCLift());
		addSequential(new DriveForwardBackwardDistance(.2, 0, 30));
		addParallel(new DriveForwardBackwardDistance(.1, 0, 10));
		addSequential(new RaiseToteLift(), 1);
		addParallel(new RaiseToteLift(), 1);
		addSequential(new TurnLeftToAngle());
		addSequential(new DriveForwardBackwardDistance(.3, 0, 120));// was 110
		addSequential(new TurnLeftToAngle());

	}
}
