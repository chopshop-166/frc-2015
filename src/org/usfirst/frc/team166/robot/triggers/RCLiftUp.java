package org.usfirst.frc.team166.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Trigger;

import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.Utility;

/**
 *
 */
public class RCLiftUp extends Trigger {

	@Override
	public boolean get() {
		return Utility.isAxisZero(Robot.oi.getRCLiftUpDownAxis());
	}
}
