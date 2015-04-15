package org.usfirst.frc.team166.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.commands.claw.CloseClaw;
import org.usfirst.frc.team166.robot.commands.drive.CenterOnTote;
import org.usfirst.frc.team166.robot.commands.drive.DriveDirection;
import org.usfirst.frc.team166.robot.commands.lifts.LowerToteLift;
import org.usfirst.frc.team166.robot.commands.lifts.RaiseRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.RaiseToteLift;

/**
 *
 */
public class ToteAndRCAutonomous extends CommandGroup {

	public ToteAndRCAutonomous() {

		// NEW AUTO STRAIGHT ON BOTH NOT READY
		addParallel(new LowerToteLift());
		addSequential(new CloseClaw());
		addSequential(new RaiseRCLift(), 2);
		addParallel(new RaiseRCLift(), 2);
		addSequential(new DriveDirection(270, .3), 2); // strafe
		addSequential(new CenterOnTote());
		addSequential(new DriveForwardBackwardDistance(.15, 0, 7));
		addSequential(new RaiseToteLift(), 1);
		addParallel(new RaiseToteLift(), 1);
		addSequential(new DriveForwardBackwardDistance(.3, 180, 90));// was 100
		addSequential(new TurnLeftToAngle(90));

	}
}
