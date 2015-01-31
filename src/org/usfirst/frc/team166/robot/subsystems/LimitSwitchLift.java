package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

import org.usfirst.frc.team166.robot.RobotMap;

/**
 *
 */
public class LimitSwitchLift extends Lift {
	// Speed controller, solenoid, encoder A and B channel, stop limit switch, limit switch direction
	DigitalInput carriageLimit = new DigitalInput(RobotMap.CarriageRCLiftLimit);

	public LimitSwitchLift(int motorChannel, int brakeChannel, int encoderChannelA, int encoderChannelB,
			int boundaryLimitChannel, LimitBoundary bound) {
		super(motorChannel, brakeChannel, encoderChannelA, encoderChannelB, boundaryLimitChannel, bound);
	}

	public boolean areLiftsInContact() {
		return carriageLimit.get();
	}

}
