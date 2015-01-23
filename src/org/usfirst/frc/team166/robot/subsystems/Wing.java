package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Wing extends Subsystem {
	private Solenoid solenoid;

	public Wing(int channel) {
		solenoid = new Solenoid(channel);
	}

	public void raise() {
		solenoid.set(true);

	}

	public void lower() {
		solenoid.set(false);
	}

	@Override
	public void initDefaultCommand() {

	}
}
