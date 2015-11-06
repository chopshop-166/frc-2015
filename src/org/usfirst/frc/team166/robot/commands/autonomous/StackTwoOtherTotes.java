package org.usfirst.frc.team166.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.RobotMap;
import org.usfirst.frc.team166.robot.commands.drive.DriveToDistance;

/**
 *
 */
public class StackTwoOtherTotes extends CommandGroup {
	double autoForwardSpeed = Preferences.getInstance().getDouble(RobotMap.Prefs.AutoForwardSpeed, 0.3);

	public StackTwoOtherTotes() {
		addSequential(new DriveToDistance(.2));
		addSequential(new TurnLeftToAngle(90));
		addSequential(new StackTwoTotes());
	}
}
