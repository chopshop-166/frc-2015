package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team166.robot.Robot;
import org.usfirst.frc.team166.robot.RobotMap;

/**
 *
 */
public class LimitSwitchLift extends Lift {

	DigitalInput carriageLimit = new DigitalInput(RobotMap.Switch.CarriageRCLiftLimit);

	public LimitSwitchLift(int motorChannel, int brakeChannel, int encoderChannelA, int encoderChannelB,
			int boundaryLimitChannel, String subsystem) {
		super(motorChannel, brakeChannel, encoderChannelA, encoderChannelB, boundaryLimitChannel, subsystem);
		LiveWindow.addSensor(subsystem, "Carriage Switch", carriageLimit);
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
