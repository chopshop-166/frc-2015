package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team166.robot.RobotMap;

/**
 *
 */
public class JankShank extends Subsystem {

	enum JankStates {
		Open, Close
	}

	private JankStates jankState;
	private DoubleSolenoid solenoid = new DoubleSolenoid(RobotMap.solenoid.Pcm12, RobotMap.solenoid.JankForward,
			RobotMap.solenoid.JankReverse);

	public JankShank() {
		this.jankState = JankStates.Open; // Added by Matt since claw starts as open, not closed.
		LiveWindow.addActuator("JankShank", "Solenoid", solenoid);
	}

	// open
	public void open() {
		solenoid.set(DoubleSolenoid.Value.kForward);
		this.jankState = JankStates.Open;
	}

	// close
	public void close() {
		solenoid.set(DoubleSolenoid.Value.kReverse);
		this.jankState = JankStates.Close;
	}

	// command used by trigger V
	public void Toggle() {
		if (this.jankState == JankStates.Open) {
			close();
		} else {
			open(); // Only worked before since this is an "else" not and "else if": The claw was open to start
			// and was "opened" again (nothing happened), then the second time it would actually close.
		}
	}

	// Measures the Boolean of the claw Solenoid
	public void setState() {

		if (solenoid.get() == DoubleSolenoid.Value.kForward) {
			this.jankState = JankStates.Open;
		} else {
			this.jankState = JankStates.Close;
		}
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
