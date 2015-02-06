package org.usfirst.frc.team166.robot.triggers;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.buttons.Trigger;

import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.RobotMap;

/**
 *
 */
public class RCLiftStalled extends Trigger {

	@Override
	public boolean get() {
		return Robot.pdBoard.getCurrent(RobotMap.Power.RCLiftMotor) > Preferences.getInstance().getDouble(
				"LiftMaxCurrent", 20);
	}
}
