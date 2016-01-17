package org.usfirst.frc.team166.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Trigger;

import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.RobotMap;

/**
 *
 */
public class ToteLiftStalled extends Trigger {

	@Override
	public boolean get() {
		// return Robot.pdBoard.getCurrent(RobotMap.Power.ToteLiftMotor) > Preferences.getInstance().getDouble(
		// RobotMap.Prefs.LiftMaxCurrent, 20)
		// || Robot.rcLift.isMotorStuck(Robot.rcLift);
		// return Robot.pdBoard.getCurrent(RobotMap.Power.ToteLiftMotor) > Preferences.getInstance().getDouble(
		// RobotMap.Prefs.LiftMaxCurrent, 20);
		return (Robot.pdBoard.getCurrent(RobotMap.Power.ToteLiftMotor) > 9001);
	}
}
