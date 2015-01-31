package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team166.robot.Robot;

/**
 *
 */
public class Lift extends Subsystem {
	// Speed controller, solenoid, encoder A and B channel, stop limit switch, limit switch direction
	DigitalInput boundaryLimit;
	Talon motor;
	Encoder encoder;
	Solenoid brake;
	LimitBoundary limitPosition;

	public enum LimitBoundary {
		Top, Bottom
	}

	public Lift(int motorChannel, int brakeChannel, int encoderChannelA, int encoderChannelB, int boundaryLimitChannel,
			LimitBoundary bound) {
		motor = new Talon(motorChannel);
		brake = new Solenoid(brakeChannel);
		encoder = new Encoder(encoderChannelA, encoderChannelB);
		boundaryLimit = new DigitalInput(boundaryLimitChannel);
		limitPosition = bound;
	}

	public void move() {
		// Robot.oi.getRCJoystick();
		releaseLift();
		if (Robot.oi.getToteJoystick().getX() < Preferences.getInstance().getDouble("Lift Deadzone", 0)) {
			motor.set(Preferences.getInstance().getDouble("Move Speed", 0));
		} else if ((Robot.oi.getToteJoystick().getX() > Preferences.getInstance().getDouble("Lift Deadzone", 0))) {
			motor.set(-Preferences.getInstance().getDouble("Move Speed", 0));
		} else {
			brakeLift();
		}
	}

	public void brakeLift() {
		brake.set(true);
	}

	public void releaseLift() {
		brake.set(false);
	}

	public void resetEncoder() {
	}

	public boolean getBotLiftLimit() {
		return boundaryLimit.get();
	}

	public void stop() {

		return;
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
