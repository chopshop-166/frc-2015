package org.usfirst.frc.team166.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team166.robot.Robot;

/**
 *
 */
public class RCLiftUpTrig extends Trigger {

	@Override
	public boolean get() {
		SmartDashboard.putNumber("RCLift up-down axis", Robot.oi.getRCLiftUpDownAxis());
		return (Robot.oi.getRCLiftUpDownAxis() < 0);
	}
}
