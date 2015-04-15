package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team166.robot.RobotMap;

/**
 *
 */
public class JankShank extends Subsystem {

	enum JankState {
		Open, Close
	}

	private JankState jankState;
	private DoubleSolenoid solenoid = new DoubleSolenoid(RobotMap.solenoid.Pcm12, RobotMap.solenoid.JankForward,
			RobotMap.solenoid.JankReverse);

	public JankShank() {
		this.jankState = JankState.Open; // Added by Matt since claw starts as open, not closed.
		// LiveWindow.addActuator("Claw", "Solenoid", solenoid);
	}

	// open
	public void open() {
		solenoid.set(DoubleSolenoid.Value.kForward);
		this.jankState = JankState.Open;
	}

	// close
	public void close() {
		solenoid.set(DoubleSolenoid.Value.kReverse);
		this.jankState = JankState.Close;
	}

	// command used by trigger V
	public void Toggle() {
		if (this.jankState == JankState.Open) {
			close();
		} else {
			open(); // Only worked before since this is an "else" not and "else if": The claw was open to start
			// and was "opened" again (nothing happened), then the second time it would actually close.
		}
	}

	// Measures the Boolean of the claw Solenoid
	public void setState() {

		if (solenoid.get() == DoubleSolenoid.Value.kForward) {
			this.jankState = JankState.Open;
		} else {
			this.jankState = JankState.Close;
		}
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
