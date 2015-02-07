package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;

import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.RobotMap;

/**
 *
 */
public class LimitSwitchLift extends Lift {
	// Speed controller, solenoid, encoder A and B channel, stop limit switch, limit switch direction
	DigitalInput carriageLimit = new DigitalInput(RobotMap.Switch.CarriageRCLiftLimit);

	public LimitSwitchLift(int motorChannel, int brakeChannel, int encoderChannelA, int encoderChannelB,
			int boundaryLimitChannel, LimitBoundary bound) {
		super(motorChannel, brakeChannel, encoderChannelA, encoderChannelB, boundaryLimitChannel, bound);
		encoder.setDistancePerPulse(Preferences.getInstance().getDouble(RobotMap.Prefs.RCLiftDistPerPulse, 0));
	}

	public boolean areLiftsInContact() {
		return carriageLimit.get();
	}

	@Override
	public boolean isLiftStalled() {
		return Robot.pdBoard.getCurrent(RobotMap.Power.RCLiftMotor) > Preferences.getInstance().getDouble(
				"LiftMaxCurrent", 20);
	}

}
