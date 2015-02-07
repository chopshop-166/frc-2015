package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class Wing extends Subsystem {
	private Solenoid solenoid;

	public Wing(String subsystem, int channel) {
		solenoid = new Solenoid(channel);
		LiveWindow.addActuator(subsystem, "Solenoid", solenoid);
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
