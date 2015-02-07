package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team166.robot.PIDSpeedController;
import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.RobotMap;

/**
 *
 */
public class Lift extends Subsystem {

	DigitalInput boundaryLimit;
	Talon motor;
	Encoder encoder;
	Solenoid brake;
	LiftMovement movementState;
	PIDSpeedController pid;

	// This enum describes the movement state of a lift.
	public enum LiftMovement {
		Stopped, Up, Down
	}

	// This enum describes which carriage is pushing during a collision
	public enum WhichCarriagePushing {
		RC, Tote, None, Both
	}

	// Constructor
	public Lift(int motorChannel, int brakeChannel, int encoderChannelA, int encoderChannelB, int boundaryLimitChannel,
			String subsystem) {
		motor = new Talon(motorChannel);
		brake = new Solenoid(brakeChannel);
		encoder = new Encoder(encoderChannelA, encoderChannelB);
		boundaryLimit = new DigitalInput(boundaryLimitChannel);

		encoder.setDistancePerPulse(Preferences.getInstance().getDouble(RobotMap.Prefs.ToteLiftDistPerPulse, 0));

		LiveWindow.addActuator(subsystem, "Motor", motor);
		LiveWindow.addActuator(subsystem, "Brake", brake);
		LiveWindow.addSensor(subsystem, "Encoder", encoder);
		LiveWindow.addSensor(subsystem, "Boundary Limit Switch", boundaryLimit);
		pid = new PIDSpeedController(encoder, motor, subsystem, "Speed Control");

	}

	public void moveUp() {
		releaseBrake();
		pid.set(getLiftSpeed());
	}

	public void moveDown() {
		releaseBrake();
		pid.set(-getLiftSpeed());
	}

	public void stop() {
		pid.set(0);
		setBrake();
	}

	// Move lift to given position
	public void moveLiftToPosition(double position) {
		if (encoder.getDistance() > position + Preferences.getInstance().getDouble(RobotMap.Prefs.LiftPosTolerance, 10)) {
			pid.set(-getLiftSpeed());
		} else if (encoder.getDistance() < position
				- Preferences.getInstance().getDouble(RobotMap.Prefs.LiftPosTolerance, 10)) {
			pid.set(getLiftSpeed());
		} else {
			stop();
		}
	}

	public boolean isAtTargetPos(double position) {
		return (encoder.getDistance() < position
				+ Preferences.getInstance().getDouble(RobotMap.Prefs.LiftPosTolerance, 10) && encoder.getDistance() > position
				- Preferences.getInstance().getDouble(RobotMap.Prefs.LiftPosTolerance, 10));
	}

	// Given lift move states, decides which carriage is pushing in a collision, and sets WhichCarriageMoving
	public static WhichCarriagePushing collisionMovement(LiftMovement rcMoveState, LiftMovement toteMoveState) {
		if (rcMoveState == LiftMovement.Stopped && toteMoveState == LiftMovement.Up)
			return WhichCarriagePushing.Tote;
		else if (rcMoveState == LiftMovement.Down && toteMoveState == LiftMovement.Stopped)
			return WhichCarriagePushing.RC;
		else if (rcMoveState == LiftMovement.Down && toteMoveState == LiftMovement.Up)
			return WhichCarriagePushing.Both;
		else
			return WhichCarriagePushing.None;
	}

	// Set Speed PID constants from preferences
	public void liftPIDInit() {
		double p = Preferences.getInstance().getDouble(RobotMap.Prefs.LiftSpeedP, 0);
		double i = Preferences.getInstance().getDouble(RobotMap.Prefs.LiftSpeedI, 0);
		double d = Preferences.getInstance().getDouble(RobotMap.Prefs.LiftSpeedD, 0);
		double f = Preferences.getInstance().getDouble(RobotMap.Prefs.LiftSpeedF, 0);

		pid.setConstants(p, i, d, f);
	}

	// Returns whether or not the lift boundary limit switch is hit
	public boolean isBoundaryHit() {
		return boundaryLimit.get();
	}

	// Returns whether or not the lift motor is drawing too much current
	public boolean isLiftStalled() {
		return Robot.pdBoard.getCurrent(RobotMap.Power.ToteLiftMotor) > Preferences.getInstance().getDouble(
				RobotMap.Prefs.LiftMaxCurrent, 20);
	}

	public LiftMovement getMoveState() {
		return movementState;
	}

	// Activate brake
	private void setBrake() {
		brake.set(true);
	}

	// Deactivate brake
	public void releaseBrake() {
		brake.set(false);
	}

	public void resetEncoder() {
		encoder.reset();
	}

	// Get the max of the preference and zero so a negative doesn't change directions
	private double getLiftSpeed() {
		return Math.max(Preferences.getInstance().getDouble(RobotMap.Prefs.LiftSpeed, 0), 0);
	}

	@Override
	public void initDefaultCommand() {

	}
}
