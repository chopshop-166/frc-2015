package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team166.robot.RobotMap;

/**
 *
 */
public class Claw extends Subsystem {

	enum ClawStates {
		Open, Close
	}

	private ClawStates clawState;
	private DoubleSolenoid solenoid = new DoubleSolenoid(RobotMap.solenoid.Pcm24, RobotMap.solenoid.ClawForward,
			RobotMap.solenoid.ClawReverse);

	public Claw() {
		this.clawState = ClawStates.Open; // Added by Matt since claw starts as open, not closed.
		LiveWindow.addActuator("Claw", "Solenoid", solenoid);
	}

	// open
	public void open() {
		solenoid.set(DoubleSolenoid.Value.kForward);
		this.clawState = ClawStates.Open;
	}

	// close
	public void close() {
		solenoid.set(DoubleSolenoid.Value.kReverse);
		this.clawState = ClawStates.Close;
	}

	// command used by trigger V
	public void Toggle() {
		if (this.clawState == ClawStates.Open) {
			close();
		} else {
			open(); // Only worked before since this is an "else" not and "else if": The claw was open to start
			// and was "opened" again (nothing happened), then the second time it would actually close.
		}
	}

	// Measures the Boolean of the claw Solenoid
	public void setState() {

		if (solenoid.get() == DoubleSolenoid.Value.kForward) {
			this.clawState = ClawStates.Open;
		} else {
			this.clawState = ClawStates.Close;
		}
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
