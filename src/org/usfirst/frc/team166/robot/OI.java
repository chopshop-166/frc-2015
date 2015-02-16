package org.usfirst.frc.team166.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team166.robot.commands.claw.CloseClaw;
import org.usfirst.frc.team166.robot.commands.claw.OpenClaw;
import org.usfirst.frc.team166.robot.commands.claw.ToggleClaw;
import org.usfirst.frc.team166.robot.commands.drive.CancelDriveCommand;
import org.usfirst.frc.team166.robot.commands.drive.DriveDirection;
import org.usfirst.frc.team166.robot.commands.lifts.LowerRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.LowerToteLift;
import org.usfirst.frc.team166.robot.commands.lifts.RaiseRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.RaiseToteLift;
import org.usfirst.frc.team166.robot.commands.lifts.ReleaseLiftBrake;
import org.usfirst.frc.team166.robot.commands.lifts.StopRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.StopToteLift;
import org.usfirst.frc.team166.robot.commands.wings.LiftWings;
import org.usfirst.frc.team166.robot.commands.wings.LowerWings;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands and command groups
 * that allow control of the robot.
 */
public class OI {

	private final Joystick driveJoystick;
	private final Joystick copilotController;

	public OI() {

		driveJoystick = new Joystick(RobotMap.DriveJoystick);
		copilotController = new Joystick(RobotMap.CopilotController);
		JoystickButton button3 = new JoystickButton(driveJoystick, 3);
		JoystickButton button4 = new JoystickButton(driveJoystick, 4);
		button3.whileHeld(new DriveDirection(270, Preferences.getInstance().getDouble("StrafePower", .25)));
		button4.whileHeld(new DriveDirection(90, Preferences.getInstance().getDouble("StrafePower", .25)));

		// Wing commands
		SmartDashboard.putData("LiftWings", new LiftWings());
		SmartDashboard.putData("LowerWings", new LowerWings());

		// Claw commands
		SmartDashboard.putData("Toggle Claw", new ToggleClaw());
		SmartDashboard.putData("Open claw", new OpenClaw());
		SmartDashboard.putData("Close claw", new CloseClaw());

		// Lift commands
		SmartDashboard.putData("Release toteLift brake", new ReleaseLiftBrake(Robot.toteLift));
		SmartDashboard.putData("Release rcLift brake", new ReleaseLiftBrake(Robot.rcLift));
		SmartDashboard.putData("Stop toteLift", new StopToteLift());
		SmartDashboard.putData("Stop rcLift", new StopRCLift());
		SmartDashboard.putData("Move toteLift up", new RaiseToteLift());
		SmartDashboard.putData("Move rcLift up", new RaiseRCLift());
		SmartDashboard.putData("Move toteLift down", new LowerToteLift());
		SmartDashboard.putData("Move rcLift down", new LowerRCLift());

		// Drive commands
		SmartDashboard.putData("Cancel drive command", new CancelDriveCommand());
		SmartDashboard.putData("StrafeRight", new DriveDirection(90, .35));
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

	public double getRightXboxTrigger() {
		return (copilotController.getRawAxis(RobotMap.RightXboxTrigger));
	}
}
