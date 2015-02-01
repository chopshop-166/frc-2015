package org.usfirst.frc.team166.robot.triggers;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.buttons.Trigger;

import org.usfirst.frc.team166.robot.Robot;

/**
 *
 */
public class ToteLiftDown extends Trigger {

	@Override
	public boolean get() {
		return Robot.oi.getToteJoystick().getY() < -Preferences.getInstance().getDouble("Deadzone", .2);
	}
}
