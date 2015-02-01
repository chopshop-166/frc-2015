package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team166.robot.PIDSpeedController;

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

	PIDSpeedController pid;

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

	public void moveUp() {
		releaseLift();
		pid.set(Preferences.getInstance().getDouble("Move Speed", 0));
	}

	public void moveDown() {
		releaseLift();
		pid.set(-Preferences.getInstance().getDouble("Move Speed", 0));
	}

	public void stop() {
		pid.set(0);
		brakeLift();
	}

	public static WhichCarriagePushing collisionMovement(LiftMovement rcMoveState, LiftMovement toteMoveState) {
		if (rcMoveState == LiftMovement.Stopped && toteMoveState == LiftMovement.Up)
			return WhichCarriagePushing.Tote;
		else if (rcMoveState == LiftMovement.Down && toteMoveState == LiftMovement.Stopped)
			return WhichCarriagePushing.RC;
		else
			return WhichCarriagePushing.None;
	}

	public void liftPIDInit(String liftName, String controllerName) {
		double p = Preferences.getInstance().getDouble("Lift P", 0);
		double i = Preferences.getInstance().getDouble("Lift I", 0);
		double d = Preferences.getInstance().getDouble("Lift D", 0);
		double f = Preferences.getInstance().getDouble("Lift F", 0);

		pid = new PIDSpeedController(encoder, motor, liftName, controllerName);
		pid.setConstants(p, i, d, f);
	}

	public boolean isBoundaryHit() {
		return boundaryLimit.get();
	}

	public void bePushed() {
		motor.set(Preferences.getInstance().getDouble("Move Speed", 0));
	}

	public LiftMovement getMoveState() {
		return movementState;
	}

	private void brakeLift() {
		brake.set(true);
	}

	public void releaseLift() {
		brake.set(false);
	}

	public void resetEncoder() {
		encoder.reset();
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
