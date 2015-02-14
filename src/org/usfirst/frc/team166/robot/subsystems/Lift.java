package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team166.robot.PIDSpeedController;
import org.usfirst.frc.team166.robot.RobotMap;

/**
 *
 */
public class Lift extends Subsystem {

	DigitalInput boundaryLimit;
	Talon motor;
	Encoder encoder;
	DoubleSolenoid brake;
	LiftMovement movementState;
	PIDSpeedController pid;
	String subsystemName;

	// This enum describes the movement state of a lift.
	public enum LiftMovement {
		Stopped, Up, Down
	}

	// This enum describes which carriage is pushing during a collision
	public enum WhichCarriagePushing {
		RC, Tote, None, Both
	}

	// Constructor
	public Lift(int motorChannel, int brakeChannelForward, int brakeChannelReverse, int encoderChannelA,
			int encoderChannelB, int boundaryLimitChannel, String subsystem) {
		motor = new Talon(motorChannel);
		brake = new DoubleSolenoid(RobotMap.solenoid.Pcm24, brakeChannelForward, brakeChannelReverse);
		encoder = new Encoder(encoderChannelA, encoderChannelB);
		boundaryLimit = new DigitalInput(boundaryLimitChannel);

		LiveWindow.addActuator(subsystem, "Motor", motor);
		LiveWindow.addActuator(subsystem, "Brake", brake);
		LiveWindow.addSensor(subsystem, "Encoder", encoder);
		LiveWindow.addSensor(subsystem, "Boundary Limit Switch", boundaryLimit);

		encoder.setPIDSourceParameter(PIDSourceParameter.kRate);
		pid = new PIDSpeedController(encoder, motor, subsystem, "Speed Control");
		subsystemName = subsystem;
	}

	public void moveUp() {
		releaseBrake();
		pid.set(-getLiftSpeed());
	}

	public void moveDown() {
		releaseBrake();
		pid.set(getLiftSpeed());
	}

	public void stop() {
		pid.set(0);
		setBrake();
	}

	// Move lift to given position
	public void moveLiftToPosition(double position) {
		double tolerance = Preferences.getInstance().getDouble(RobotMap.Prefs.LiftPosTolerance, 10);

		if (encoder.getDistance() > position + tolerance) {
			pid.set(-getLiftSpeed());
		} else if (encoder.getDistance() < position - tolerance) {
			pid.set(getLiftSpeed());
		}
	}

	public boolean isAtTargetPos(double position) {
		double tolerance = Preferences.getInstance().getDouble(RobotMap.Prefs.LiftPosTolerance, 10);

		return (encoder.getDistance() < position + tolerance && encoder.getDistance() > position - tolerance);
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
	public void initLift() {
		double p = Preferences.getInstance().getDouble(subsystemName + RobotMap.Prefs.LiftSpeedP, 0);
		double i = Preferences.getInstance().getDouble(subsystemName + RobotMap.Prefs.LiftSpeedI, 0);
		double d = Preferences.getInstance().getDouble(subsystemName + RobotMap.Prefs.LiftSpeedD, 0);
		double f = Preferences.getInstance().getDouble(subsystemName + RobotMap.Prefs.LiftSpeedF, 0);

		pid.setConstants(p, i, d, f);

		encoder.setDistancePerPulse(Preferences.getInstance().getDouble(
				subsystemName + RobotMap.Prefs.LiftDistPerPulse, 1));
		setBrake();

	}

	// Returns whether or not the lift boundary limit switch is hit
	public boolean isBoundaryHit() {
		return boundaryLimit.get();
	}

	public LiftMovement getMoveState() {
		return movementState;
	}

	// Activate brake
	private void setBrake() {
		brake.set(DoubleSolenoid.Value.kReverse);
	}

	// Deactivate brake
	public void releaseBrake() {
		brake.set(DoubleSolenoid.Value.kForward);
	}

	public void resetEncoder() {
		encoder.reset();
	}

	// Get the max of the preference and zero so a negative doesn't change directions
	private double getLiftSpeed() {
		if (subsystemName == "Tote") {
			return -Math.max(Preferences.getInstance().getDouble(RobotMap.Prefs.LiftSpeed, 0), 0);
		} else
			return Math.max(Preferences.getInstance().getDouble(RobotMap.Prefs.LiftSpeed, 0), 0);
	}

	private void setInvert() {

	}

	@Override
	public void initDefaultCommand() {

	}
}
