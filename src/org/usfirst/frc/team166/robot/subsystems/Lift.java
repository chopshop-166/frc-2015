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
	String subsytemName;

	public enum LiftMovement {
		Stopped, Up, Down
	}

	public enum WhichCarriagePushing {
		RC, Tote, None, Both
	}

	public Lift(int motorChannel, int brakeChannel, int encoderChannelA, int encoderChannelB, int boundaryLimitChannel,
			String subsytem) {
		motor = new Talon(motorChannel);
		brake = new Solenoid(brakeChannel);
		encoder = new Encoder(encoderChannelA, encoderChannelB);
		boundaryLimit = new DigitalInput(boundaryLimitChannel);
		subsytemName = subsytem;

		encoder.setDistancePerPulse(Preferences.getInstance().getDouble(RobotMap.Prefs.ToteLiftDistPerPulse, 0));

		LiveWindow.addActuator(subsytem, "Motor", motor);
		LiveWindow.addActuator(subsytem, "Brake", brake);
		LiveWindow.addSensor(subsytem, "Encoder", encoder);
		LiveWindow.addSensor(subsytem, "Boundary Limit Switch", boundaryLimit);
		pid = new PIDSpeedController(encoder, motor, subsytemName, "Speed Control");
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

	public void liftPIDInit() {
		double p = Preferences.getInstance().getDouble(RobotMap.Prefs.LiftP, 0);
		double i = Preferences.getInstance().getDouble(RobotMap.Prefs.LiftI, 0);
		double d = Preferences.getInstance().getDouble(RobotMap.Prefs.LiftD, 0);
		double f = Preferences.getInstance().getDouble(RobotMap.Prefs.LiftF, 0);

		pid.setConstants(p, i, d, f);
	}

	public boolean isBoundaryHit() {
		return boundaryLimit.get();
	}

	public boolean isLiftStalled() {
		return Robot.pdBoard.getCurrent(RobotMap.Power.ToteLiftMotor) > Preferences.getInstance().getDouble(
				RobotMap.Prefs.LiftMaxCurrent, 20);
	}

	public LiftMovement getMoveState() {
		return movementState;
	}

	private void setBrake() {
		brake.set(true);
	}

	public void releaseBrake() {
		brake.set(false);
	}

	public void resetEncoder() {
		encoder.reset();
	}

	private double getLiftSpeed() {
		// Get the max of the preference and zero so a negative doesn't change directions
		return Math.max(Preferences.getInstance().getDouble(RobotMap.Prefs.LiftSpeed, 0), 0);
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
