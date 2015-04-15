package org.usfirst.frc.team166.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.commands.claw.CloseClaw;
import org.usfirst.frc.team166.robot.commands.lifts.LowerToteLift;
import org.usfirst.frc.team166.robot.commands.lifts.RaiseRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.RaiseToteLift;

/**
 *
 */
public class ToteOrRCAutonomous extends CommandGroup {

	public ToteOrRCAutonomous() {

		// RC OR TOTE AUTO
		addParallel(new LowerToteLift());
		addSequential(new CloseClaw());
		addSequential(new RaiseRCLift(), 1.66); // was 2
		addSequential(new DriveForwardBackwardDistance(.1, 0, 2));
		addParallel(new DriveForwardBackwardDistance(.1, 0, 2));
		addSequential(new RaiseToteLift(), 1);// was 2
		addSequential(new DriveForwardBackwardDistance(.3, 180, 90));// used to be 100
		addSequential(new TurnLeftToAngle(90));

	}
}
