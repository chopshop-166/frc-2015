package org.usfirst.frc.team166.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team166.robot.RobotMap;
import org.usfirst.frc.team166.robot.commands.drive.DriveDirection;
import org.usfirst.frc.team166.robot.commands.groups.InitLiftEncoders;
import org.usfirst.frc.team166.robot.commands.wings.RaiseLeftWing;
import org.usfirst.frc.team166.robot.commands.wings.RaiseRightWing;

/**
 *
 */
public class StepRCAutonomous extends CommandGroup {
	double autoForwardSpeed = Preferences.getInstance().getDouble(RobotMap.Prefs.AutoForwardSpeed, 0.3);

	public StepRCAutonomous() {
		addParallel(new InitLiftEncoders());
		addSequential(new DriveToStep());
		addSequential(new CenterOnStep(), 3.0);
		addSequential(new DriveDirection(0, Preferences.getInstance().getDouble("DriveAngleSpeed", 0)), .5);
		addSequential(new RaiseLeftWing());
		addParallel(new RaiseRightWing());
		// addSequential(new DriveForwardBackwardDistance(autoForwardSpeed, ForwardBackwardDirection.Forward,
		// Preferences
		// .getInstance().getInt(RobotMap.Prefs.AutoForwardDistance1, 40)));
		// addSequential(new AutonomousStrafe(-.25));
		// addSequential(new DriveForwardBackwardDistance(autoForwardSpeed, ForwardBackwardDirection.Forward,
		// Preferences
		// .getInstance().getInt(RobotMap.Prefs.AutoForwardDistance2, 50)));
	}
}
