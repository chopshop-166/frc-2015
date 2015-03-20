package org.usfirst.frc.team166.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.commands.claw.CloseClaw;
import org.usfirst.frc.team166.robot.commands.claw.OpenClaw;
import org.usfirst.frc.team166.robot.commands.drive.DriveDirection;
import org.usfirst.frc.team166.robot.commands.lifts.LowerToteLift;
import org.usfirst.frc.team166.robot.commands.lifts.RaiseRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.RaiseToteLift;

/**
 *
 */
public class RCToteAutonomous extends CommandGroup {

	public RCToteAutonomous() {
		// OLD RC AND TOTE AUTO THAT INTERFERES TESTED
		addSequential(new OpenClaw());
		addParallel(new LowerToteLift());
		addSequential(new CloseClaw());
		addSequential(new RaiseRCLift(), 2);
		addParallel(new RaiseRCLift(), 2);
		addSequential(new DriveForwardBackwardDistance(.2, 0, 30));
		addParallel(new DriveForwardBackwardDistance(.1, 0, 10));
		addSequential(new RaiseToteLift(), 1);
		addParallel(new RaiseToteLift(), 1);
		addSequential(new TurnLeftToAngle(90));
		addSequential(new DriveForwardBackwardDistance(.3, 0, 120));
		addSequential(new TurnLeftToAngle(90));

		// NEW RC AUTO THAT DOES NOT INTERFERE NOT TESTED
		addSequential(new OpenClaw());
		addParallel(new LowerToteLift());
		addSequential(new CloseClaw());
		addSequential(new RaiseRCLift(), 2);
		addSequential(new DriveForwardBackwardDistance(.2, 0, 5));
		addSequential(new RaiseToteLift(), 2);
		addSequential(new DriveForwardBackwardDistance(.3, 180, 125));
		addSequential(new TurnLeftToAngle(90));

		// NEW RC AND TOTE AUTO THAT DOES NOT INTERFERE UNTESTED
		addSequential(new OpenClaw());
		addParallel(new LowerToteLift());
		addSequential(new CloseClaw());
		addSequential(new RaiseRCLift(), 2);
		addParallel(new RaiseRCLift(), 2);
		addSequential(new DriveForwardBackwardDistance(.2, 0, 10));
		addSequential(new TurnLeftToAngle(45));
		addSequential(new DriveDirection(.3, 90), .5);
		addParallel(new DriveForwardBackwardDistance(.1, 0, 10));
		addSequential(new RaiseToteLift(), 1);
		addParallel(new RaiseToteLift(), 1);
		addSequential(new TurnLeftToAngle(90));
		addSequential(new DriveForwardBackwardDistance(.3, 0, 120));
		addSequential(new TurnLeftToAngle(90));

	}
}
