package org.usfirst.frc.team166.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Trigger;

import org.usfirst.frc.team166.robot.Robot;

/**
 *
 */
public class CarriageTrigger extends Trigger {

	@Override
	public boolean get() {
		return Robot.rcLift.areLiftsInContact();
	}
}
