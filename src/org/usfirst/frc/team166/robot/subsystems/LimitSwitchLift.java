package org.usfirst.frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team166.robot.RobotMap;

/**
 *
 */
public class LimitSwitchLift extends Lift {

	DigitalInput carriageLimit = new DigitalInput(RobotMap.Switch.CarriageRCLiftLimit);

	public LimitSwitchLift(int motorChannel, int brakeChannelForward, int brakeChannelReverse, int encoderChannelA,
			int encoderChannelB, int boundaryLimitChannel, String subsystem) {
		super(motorChannel, brakeChannelForward, brakeChannelReverse, encoderChannelA, encoderChannelB,
				boundaryLimitChannel, subsystem);
		LiveWindow.addSensor(subsystem, "Carriage Switch", carriageLimit);
	}

	public boolean areLiftsInContact() {
		return !carriageLimit.get();
	}

	@Override
	public void initLift() {
		double p = Preferences.getInstance().getDouble(subsystemName + RobotMap.Prefs.LiftSpeedP, 0);
		double i = Preferences.getInstance().getDouble(subsystemName + RobotMap.Prefs.LiftSpeedI, 0);
		double d = Preferences.getInstance().getDouble(subsystemName + RobotMap.Prefs.LiftSpeedD, 0);
		double f = Preferences.getInstance().getDouble(subsystemName + RobotMap.Prefs.LiftSpeedF, 0);

		pid.setConstants(.1, .75, 0, .95);

		// encoder.setDistancePerPulse(Preferences.getInstance().getDouble(
		// subsystemName + RobotMap.Prefs.LiftDistPerPulse, .000611111));
		encoder.setDistancePerPulse(0.000143); // 21 was max speed
		setBrake();
	}
}
