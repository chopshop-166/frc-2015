package org.usfirst.frc.team166.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class PIDSpeedController implements SpeedController {

	PIDController controller;

	public PIDSpeedController(PIDSource source, PIDOutput output, String controllerName) {
		// controller = new PIDController(Preferences.getInstance().getDouble("P",
		// 0),Preferences.getInstance().getDouble("I", 0),
		// Preferences.getInstance().getDouble("D", 0),Preferences.getInstance().getDouble("F", 0),source,output);
		controller = new PIDController(2, 0, 0, 1, source, output);
		// LiveWindow.addActuator("Drive", "FrontLeft", controller);
		LiveWindow.addActuator("Drive", controllerName, controller);

	}

	public void setConstants() {
		// controller.setPID(Preferences.getInstance().getDouble("P", 0),Preferences.getInstance().getDouble("I", 0),
		// Preferences.getInstance().getDouble("D", 0),Preferences.getInstance().getDouble("F", 0));
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		// Never Used

	}

	@Override
	public double get() {

		// TODO Auto-generated method stub
		return controller.getSetpoint();
	}

	@Override
	public void set(double setpoint, byte syncGroup) {
		controller.setSetpoint(setpoint);
		controller.enable();
	}

	@Override
	public void set(double setpoint) {
		controller.setSetpoint(setpoint);
		controller.enable();
		// TODO Auto-generated method stub

	}

	@Override
	public void disable() {
		controller.disable();
		// TODO Auto-generated method stub

	}

	public void reset() {
		controller.reset();
	}

}
