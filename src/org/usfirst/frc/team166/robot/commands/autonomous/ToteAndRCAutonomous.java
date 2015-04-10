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
		// addSequential(new TurnLeftToAngle(90));
		// addSequential(new DriveForwardBackwardDistance(.3, 0, 120));
		// addSequential(new TurnLeftToAngle(90));

		// NEW RC OR TOTE AUTO THAT DOES NOT INTERFERE READY
		// addParallel(new LowerToteLift());
		// addSequential(new CloseClaw());
		// addSequential(new RaiseRCLift(), 1.66); // was 2
		// addSequential(new DriveForwardBackwardDistance(.1, 0, 2));
		// addParallel(new DriveForwardBackwardDistance(.1, 0, 2));
		// addSequential(new RaiseToteLift(), 1);// was 2
		// addSequential(new DriveForwardBackwardDistance(.3, 180, 90));// used to be 100
		// addSequential(new TurnLeftToAngle(90));

		// 45 DEGREE RC AND TOTE
		// addSequential(new OpenClaw());
		// addParallel(new LowerToteLift());
		// addSequential(new CloseClaw());
		// addSequential(new RaiseRCLift(), 2);
		// addParallel(new RaiseRCLift(), 2);
		// addSequential(new DriveForwardBackwardDistance(.2, 0, 10));
		// addSequential(new TurnLeftToAngle(45));
		// addSequential(new DriveDirection(90, .3), 1.05);
		// addSequential(new DriveForwardBackwardDistance(.15, 0, 7));
		// addParallel(new DriveForwardBackwardDistance(.15, 0, 5));
		// addSequential(new RaiseToteLift(), 1);
		// addParallel(new RaiseToteLift(), 1);
		// addSequential(new TurnLeftToAngle(90));
		// addSequential(new DriveForwardBackwardDistance(.3, 0, 120));
		// addSequential(new TurnLeftToAngle(90));

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

		// GG AUTO TOO STRONK
		// addSequential(new CenterOnTote());
		// addSequential(new RaiseToteLift(), .5);
		// addParallel(new RaiseToteLift(), .5);
		// addSequential(new DriveForwardBackwardDistance(.3, 180, 15));
		// addSequential(new TurnRightToAngle(90));
		// addSequential(new DriveForwardBackwardDistance(.4, 0, 40));// change that distance
		// addSequential(new TurnLeftToAngle(90));
		// addSequential(new CenterOnTote());
		// addSequential(new DriveForwardBackwardDistance(.4, 180, 2));
		// addParallel(new LowerToteLift());
		// addSequential(new DriveForwardBackwardDistance(.4, 0, 2));
		// addSequential(new RaiseToteLift(), .5);
		// addParallel(new RaiseToteLift(), .5);
		// addSequential(new DriveForwardBackwardDistance(.3, 180, 15));
		// addSequential(new TurnRightToAngle(90));
		// addSequential(new DriveForwardBackwardDistance(.4, 0, 40));// change that distance
		// addSequential(new TurnLeftToAngle(90));
		// addSequential(new CenterOnTote());
		// addSequential(new DriveForwardBackwardDistance(.4, 180, 2));
		// addParallel(new LowerToteLift());
		// addSequential(new DriveForwardBackwardDistance(.4, 0, 2));
		// addSequential(new RaiseToteLift(), .5);
		// addSequential(new DriveForwardBackwardDistance(.6, 180, 90));
		// addParallel(new LowerToteLift());
		// addSequential(new DriveForwardBackwardDistance(.3, 180, 3));

	}
}
