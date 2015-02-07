package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
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
	private Solenoid solenoid = new Solenoid(RobotMap.solenoid.ClawSolenoid);

	public Claw() {
		LiveWindow.addActuator("Claw", "Solenoid", solenoid);
	}

	// open
	public void open() {
		solenoid.set(true);
		this.clawState = ClawStates.Open;
	}

	// close
	public void close() {
		solenoid.set(false);
		this.clawState = ClawStates.Close;
	}

	// command used by trigger V
	public void Toggle() {
		if (this.clawState == ClawStates.Open) {
			close();
		} else {
			open();
		}
	}

	// Measures the Boolean of the claw Solenoid
	public void setState() {

		if (solenoid.get() == true) {
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
