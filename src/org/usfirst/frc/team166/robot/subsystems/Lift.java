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
	LiftMovement movementState;

	public enum LimitBoundary {
		Top, Bottom
	}

	public enum LiftMovement {
		Stopped, Up, Down
	}

	public enum WhichCarriagePushing {
		RC, Tote, None
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
		releaseLift();
		if (Robot.oi.getToteJoystick().getX() < Preferences.getInstance().getDouble("Lift Deadzone", 0)) {
			motor.set(Preferences.getInstance().getDouble("Move Speed", 0));
			movementState = LiftMovement.Up;
		} else if ((Robot.oi.getToteJoystick().getX() > Preferences.getInstance().getDouble("Lift Deadzone", 0))) {
			motor.set(-Preferences.getInstance().getDouble("Move Speed", 0));
			movementState = LiftMovement.Down;
		} else {
			brakeLift();
			movementState = LiftMovement.Stopped;
		}
	}

	public WhichCarriagePushing collisionMovement(LiftMovement rcMoveState, LiftMovement toteMoveState) {
		if (rcMoveState == LiftMovement.Stopped && toteMoveState == LiftMovement.Up)
			return WhichCarriagePushing.Tote;
		else if (rcMoveState == LiftMovement.Down && toteMoveState == LiftMovement.Stopped)
			return WhichCarriagePushing.RC;
		else
			return WhichCarriagePushing.None;
	}

	public void bePushed() {
		motor.set(Preferences.getInstance().getDouble("Move Speed", 0));
	}

	public LiftMovement moveState() {
		return movementState;
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
