package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class Wing extends Subsystem {
	private DoubleSolenoid solenoid;

	public Wing(String subsystem, int forwardChannel, int reverseChannel) {
		solenoid = new DoubleSolenoid(forwardChannel, reverseChannel);
		LiveWindow.addActuator(subsystem, "Solenoid", solenoid);
	}

	public void raise() {
		solenoid.set(DoubleSolenoid.Value.kForward);

	}

	public void lower() {
		solenoid.set(DoubleSolenoid.Value.kReverse);
	}

	@Override
	public void initDefaultCommand() {

	}
}
