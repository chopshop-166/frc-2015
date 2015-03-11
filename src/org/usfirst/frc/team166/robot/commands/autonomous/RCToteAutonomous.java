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
		// OLD RC AND TOTE AUTO THAT INTERFERES READY
		// addSequential(new OpenClaw());
		// addParallel(new LowerToteLift());
		// addSequential(new CloseClaw());
		// addSequential(new RaiseRCLift(), 2);
		// addParallel(new RaiseRCLift(), 2);
		// addSequential(new DriveForwardBackwardDistance(.2, 0, 30));
		// addParallel(new DriveForwardBackwardDistance(.1, 0, 10));
		// addSequential(new RaiseToteLift(), 1);
		// addParallel(new RaiseToteLift(), 1);
		// addSequential(new TurnLeftToAngle());
		// addSequential(new DriveForwardBackwardDistance(.3, 0, 120));
		// addSequential(new TurnLeftToAngle());

		// NEW RC AUTO THAT DOES NOT INTERFERE READY
		// addSequential(new OpenClaw());
		// addParallel(new LowerToteLift());
		// addSequential(new CloseClaw());
		// addSequential(new RaiseRCLift(), 2);
		// addSequential(new DriveForwardBackwardDistance(.1, 0, 2));
		// addParallel(new DriveForwardBackwardDistance(.1, 0, 2));
		// addSequential(new RaiseToteLift(), 2);
		// addSequential(new DriveForwardBackwardDistance(.3, 180, 100));
		// addSequential(new TurnLeftToAngle());

		// NEW RC AND TOTE AUTO THAT DOES NOT INTERFERE NOT READY
		addSequential(new OpenClaw());
		addParallel(new LowerToteLift());
		addSequential(new CloseClaw());
		addSequential(new RaiseRCLift(), 2);
		addParallel(new RaiseRCLift(), 2);
		addSequential(new DriveForwardBackwardDistance(.2, 0, 10));
		addSequential(new RotateLeft45());
		addSequential(new DriveDirection(90, .3), 1.05); // strafe
		addSequential(new DriveForwardBackwardDistance(.15, 0, 7));
		addParallel(new DriveForwardBackwardDistance(.15, 0, 5));
		addSequential(new RaiseToteLift(), 1);
		addParallel(new RaiseToteLift(), 1);
		addSequential(new TurnLeftToAngle());// left 90
		addSequential(new DriveForwardBackwardDistance(.3, 0, 120));
		addSequential(new TurnLeftToAngle());

		// NEW AUTO STRAIGHT ON BOTH NOT READY
		// addSequential(new OpenClaw());
		// addParallel(new LowerToteLift());
		// addSequential(new CloseClaw());
		// addSequential(new RaiseRCLift(), 2);
		// addParallel(new RaiseRCLift(), 2);
		// addSequential(new DriveDirection(270, .3), 1.25); // strafe
		// addSequential(new DriveForwardBackwardDistance(.15, 0, 28));
		// addParallel(new DriveForwardBackwardDistance(.15, 0, 7));
		// addSequential(new RaiseToteLift(), 1);
		// addParallel(new RaiseToteLift(), 1);
		// addSequential(new DriveForwardBackwardDistance(.3, 180, 130));

	}
}
