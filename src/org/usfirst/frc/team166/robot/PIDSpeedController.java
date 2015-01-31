package org.usfirst.frc.team166.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/*
 * Implements a SpeedController with an underlying PID controller
 */
public class PIDSpeedController implements SpeedController {

	PIDController controller;

	public PIDSpeedController(PIDSource source, PIDOutput output, String subsystem, String controllerName) {
		controller = new PIDController(0, 0, 0, 0, source, output);

		LiveWindow.addActuator(subsystem, controllerName, controller);
	}

	public void setConstants(double p, double i, double d, double f) {
		controller.setPID(p, i, d, f);
	}

	public void setConstants(double p, double i, double d) {
		controller.setPID(p, i, d);
	}

	@Override
	public void pidWrite(double output) {
		// Never Used
	}

	@Override
	public double get() {
		return controller.getSetpoint();
	}

	@Override
	public void set(double setpoint, byte syncGroup) {
		set(setpoint);
	}

	@Override
	public void set(double setpoint) {
		controller.setSetpoint(setpoint);
		controller.enable();
	}

	@Override
	public void disable() {
		controller.disable();
	}

	public void reset() {
		controller.reset();
	}

}
