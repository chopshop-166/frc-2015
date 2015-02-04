package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team166.robot.RobotMap;

/**
 *
 */
public class Claw extends Subsystem {

	enum ClawStates {
		open, close
	}

	ClawStates clawState;
	public static Solenoid solenoid = new Solenoid(RobotMap.solenoid.ClawSolenoid);

	public void open() {
		solenoid.set(true);
		this.clawState = ClawStates.open;
	}

	public void close() {
		solenoid.set(false);
		this.clawState = ClawStates.close;
	}

	public void Toggle() {
		if (this.clawState == ClawStates.open) {
			close();
		} else {
			open();
		}
	}

	public void begin() {
		solenoid.get();
		if (this.clawState == ClawStates.close) {
			open();
		}
	}

	public void end() {
		solenoid.get();
		if (this.clawState == ClawStates.open) {

		}
	}

	private void start() {
		// TODO Auto-generated method stub

	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
