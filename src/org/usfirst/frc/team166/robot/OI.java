package org.usfirst.frc.team166.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team166.robot.commands.claw.ToggleClaw;
import org.usfirst.frc.team166.robot.commands.drive.DriveDirection;
import org.usfirst.frc.team166.robot.commands.wings.LiftWings;
import org.usfirst.frc.team166.robot.commands.wings.LowerWings;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands and command groups
 * that allow control of the robot.
 */
public class OI {

	private final Joystick driveJoystick = new Joystick(RobotMap.DriveJoystick);
	private final Joystick copilotController = new Joystick(RobotMap.CopilotController);

	public OI() {

		JoystickButton button3 = new JoystickButton(driveJoystick, 3);
		JoystickButton button4 = new JoystickButton(driveJoystick, 4);
		button3.whileHeld(new DriveDirection(270, Preferences.getInstance().getDouble("StrafePower", .25)));
		button4.whileHeld(new DriveDirection(270, Preferences.getInstance().getDouble("StrafePower", .25)));
		SmartDashboard.putData("LiftWings", new LiftWings());
		SmartDashboard.putData("LowerWings", new LowerWings());
		SmartDashboard.putData("Toggle Claw", new ToggleClaw());
	}

	public Joystick getDriveJoystick() {
		return driveJoystick;
	}

	public double getDriveJoystickForward() {
		double axis = driveJoystick.getY();

		if (Math.abs(axis) > Preferences.getInstance().getDouble(RobotMap.Prefs.DriveDeadZone, 1)) {
			return (axis * Preferences.getInstance().getDouble(RobotMap.Prefs.DriveScalerY, 1));
		} else {
			return 0;
		}
	}

	public double getDriveJoystickLateral() {
		double axis = driveJoystick.getX();

		if (Math.abs(axis) > Preferences.getInstance().getDouble(RobotMap.Prefs.DriveDeadZone, 1)) {
			return (axis * Preferences.getInstance().getDouble(RobotMap.Prefs.DriveScalerX, 1));
		} else {
			return 0;
		}
	}

	public double getDriveJoystickRotation() {
		double axis = driveJoystick.getRawAxis(RobotMap.DriveJoystickTwistAxis);

		if (Math.abs(axis) > Preferences.getInstance().getDouble(RobotMap.Prefs.DriveDeadZone, 1)) {
			return (axis * Preferences.getInstance().getDouble(RobotMap.Prefs.DriveScalerRotation, 1));
		} else {
			return 0;
		}
	}

	public double getRCLiftUpDownAxis() {
		double axis = copilotController.getRawAxis(RobotMap.RcLiftUpDownAxis);

		if (Math.abs(axis) > Preferences.getInstance().getDouble(RobotMap.Prefs.LiftDeadzone, 1)) {
			return axis;
		} else {
			return 0;
		}
	}

	public double getToteLiftUpDownAxis() {
		double axis = copilotController.getRawAxis(RobotMap.ToteLiftUpDownAxis);

		if (Math.abs(axis) > Preferences.getInstance().getDouble(RobotMap.Prefs.LiftDeadzone, 1)) {
			return axis;
		} else {
			return 0;
		}
	}
}
