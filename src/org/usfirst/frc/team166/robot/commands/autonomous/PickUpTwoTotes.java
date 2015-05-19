package org.usfirst.frc.team166.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.RobotMap;
import org.usfirst.frc.team166.robot.commands.drive.CenterOnTote;
import org.usfirst.frc.team166.robot.commands.lifts.MoveToteLiftDistance;
import org.usfirst.frc.team166.robot.subsystems.Lift.LiftDirection;

/**
 *
 */
public class PickUpTwoTotes extends CommandGroup {
	double autoForwardSpeed = Preferences.getInstance().getDouble(RobotMap.Prefs.AutoForwardSpeed, 0.3);

	public PickUpTwoTotes() {
		// addParallel(new LowerToteLift());
		addSequential(new CenterOnTote(), 2.5);
		addSequential(new MoveToteLiftDistance(LiftDirection.up, RobotMap.PickUpToteDistance));
		addSequential(new DriveForwardBackwardDistance(.15, 0, 10));
		addSequential(new CenterOnTote(), 2.5);
		addSequential(new MoveToteLiftDistance(LiftDirection.up, RobotMap.PickUpToteDistance));
		addSequential(new DriveForwardBackwardDistance(.15, 180, 10));
	}
}
