package org.usfirst.frc.team166.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team166.robot.commands.autonomous.PickUpTwoTotes;
import org.usfirst.frc.team166.robot.commands.claw.ToggleJank;
import org.usfirst.frc.team166.robot.commands.drive.CenterOnTote;
import org.usfirst.frc.team166.robot.commands.drive.DriveDirection;
import org.usfirst.frc.team166.robot.commands.drive.MoveToDropDistance;
import org.usfirst.frc.team166.robot.commands.lifts.DecreaseToteCount;
import org.usfirst.frc.team166.robot.commands.lifts.IncreaseToteCount;
import org.usfirst.frc.team166.robot.commands.lifts.MoveToteLiftDistance;
import org.usfirst.frc.team166.robot.commands.lifts.ReleaseLiftBrake;
import org.usfirst.frc.team166.robot.commands.lifts.StopRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.StopToteLift;
import org.usfirst.frc.team166.robot.subsystems.Lift.LiftDirection;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands and command groups
 * that allow control of the robot.
 */
public class OI {

	private final Joystick driveJoystick;
	private final Joystick copilotController;

	private final JoystickButton xboxRightStickButton;
	private final JoystickButton xboxLeftStickButton;

	public OI() {

		driveJoystick = new Joystick(RobotMap.DriveJoystick);
		copilotController = new Joystick(RobotMap.CopilotController);

		JoystickButton button3 = new JoystickButton(driveJoystick, 3);
		JoystickButton button4 = new JoystickButton(driveJoystick, 4);
		JoystickButton button2 = new JoystickButton(driveJoystick, RobotMap.centerOnToteButton);
		JoystickButton driverTrigger = new JoystickButton(driveJoystick, 1);
		JoystickButton button14 = new JoystickButton(driveJoystick, 14);
		JoystickButton xboxAButton = new JoystickButton(copilotController, RobotMap.XboxAButton);
		JoystickButton xboxBButton = new JoystickButton(copilotController, RobotMap.XboxBButton);
		JoystickButton xboxXButton = new JoystickButton(copilotController, RobotMap.XboxXButton);
		JoystickButton xboxYButton = new JoystickButton(copilotController, RobotMap.XboxYButton);
		JoystickButton xboxRightBumper = new JoystickButton(copilotController, RobotMap.XboxRightBumper);

		xboxRightStickButton = new JoystickButton(copilotController, RobotMap.XboxRightStickButton);
		xboxLeftStickButton = new JoystickButton(copilotController, RobotMap.XboxLeftStickButton);

		button3.whileHeld(new DriveDirection(270, Preferences.getInstance().getDouble("StrafePower", .25)));
		button4.whileHeld(new DriveDirection(90, Preferences.getInstance().getDouble("StrafePower", .25)));
		driverTrigger.whileHeld(new CenterOnTote());
		// button2.whenPressed(new StackTwoTotes());
		button2.whenPressed(new PickUpTwoTotes());
		button14.whenPressed(new MoveToDropDistance(.1));
		xboxYButton.whenPressed(new MoveToteLiftDistance(LiftDirection.up, RobotMap.PickUpToteDistance));
		xboxAButton.whenPressed(new MoveToteLiftDistance(LiftDirection.down, RobotMap.DropTotesDistance));
		xboxRightBumper.whenPressed(new ToggleJank());
		xboxBButton.whenPressed(new IncreaseToteCount());
		xboxXButton.whenPressed(new DecreaseToteCount());

		// driverTrigger.whenPressed(new ToggleDriveSlowSpeed());

		// xboxBButton.whenPressed(new StopRCLift());
		// xboxBButton.whenPressed(new StopToteLift());

		// Wing commands
		// SmartDashboard.putData("LiftWings", new LiftWings());
		// SmartDashboard.putData("LowerWings", new LowerWings());

		// Claw commands
		// SmartDashboard.putData("Toggle Claw", new ToggleClaw())
		// SmartDashboard.putData("Open claw", new OpenClaw());
		// SmartDashboard.putData("Close claw", new CloseClaw());

		// Lift commands
		SmartDashboard.putData("Release toteLift brake", new ReleaseLiftBrake(Robot.toteLift));
		SmartDashboard.putData("Release rcLift brake", new ReleaseLiftBrake(Robot.rcLift));
		SmartDashboard.putData("Stop toteLift", new StopToteLift());
		SmartDashboard.putData("Stop rcLift", new StopRCLift());
		// SmartDashboard.putData("Move toteLift up", new RaiseToteLift());
		// SmartDashboard.putData("Move rcLift up", new RaiseRCLift());
		// SmartDashboard.putData("Move toteLift down", new LowerToteLift());
		// SmartDashboard.putData("Move rcLift down", new LowerRCLift());
		// SmartDashboard.putData("Slow move toteLiftUp", new SlowRaiseToteLift());
		// SmartDashboard.putData("Slow move rcLiftUp", new SlowRaiseRCLift());
		// SmartDashboard.putData("Slow move toteLiftDown", new SlowLowerToteLift());
		// SmartDashboard.putData("Slow move rcLiftDown", new SlowLowerRCLift());

		// Drive commands
		// SmartDashboard.putData("Cancel drive command", new CancelDriveCommand());
		// SmartDashboard.putData("StrafeRight", new DriveDirection(90, .35));
		// SmartDashboard.putData("DriveForwardBackwardDirection", new DriveForwardBackwardDistance(.2, 0, 24));
		// SmartDashboard.putData("Right RC", new RightRC());
	}

	public Joystick getDriveJoystick() {
		return driveJoystick;
	}

	public double getDriveJoystickForward() {
		double axis = driveJoystick.getY();

		if (Math.abs(axis) > Preferences.getInstance().getDouble(RobotMap.Prefs.DriveDeadZone, 1)) {
			return (axis * Robot.drive.DriveSpeedModifier * Preferences.getInstance().getDouble(
					RobotMap.Prefs.DriveScalerY, 1));
		} else {
			return 0;
		}
	}

	public double getDriveJoystickLateral() {
		double axis = driveJoystick.getX();

		if (Math.abs(axis) > Preferences.getInstance().getDouble(RobotMap.Prefs.DriveDeadZone, 1)) {
			// return (axis * Robot.drive.DriveSpeedModifier * Preferences.getInstance().getDouble(
			// RobotMap.Prefs.DriveScalerX, .5));
			return (axis * Robot.drive.DriveSpeedModifier * .5);
		} else {
			return 0;
		}
	}

	public double getDriveJoystickRotation() {
		double axis = driveJoystick.getRawAxis(RobotMap.DriveJoystickTwistAxis);
		if (Math.abs(axis) > Preferences.getInstance().getDouble(RobotMap.Prefs.DriveDeadZone, 1)) {
			// return (axis * Robot.drive.DriveSpeedModifier * Preferences.getInstance().getDouble(
			// RobotMap.Prefs.DriveScalerRotation, 1));
			return (axis * Robot.drive.DriveSpeedModifier * .4);
		} else {
			return 0;
		}
	}

	public double getDriveJoystickSlider() {
		SmartDashboard.putNumber("Joystick Slider Value",
				((-1 * driveJoystick.getRawAxis(RobotMap.DriveJoystickSliderAxis)) + 5) / 6);
		return (((-1 * driveJoystick.getRawAxis(RobotMap.DriveJoystickSliderAxis)) + 5) / 6);
	}

	public double getRCLiftUpDownAxis() {
		double axis = copilotController.getRawAxis(RobotMap.RcLiftUpDownAxis);

		if (Math.abs(axis) > Preferences.getInstance().getDouble(RobotMap.Prefs.LiftDeadzone, .25)) {
			return axis;
		} else {
			return 0;
		}
	}

	public double getToteLiftUpDownAxis() {
		double axis = copilotController.getRawAxis(RobotMap.ToteLiftUpDownAxis);

		if (Math.abs(axis) > Preferences.getInstance().getDouble(RobotMap.Prefs.LiftDeadzone, .25)) {
			return axis;
		} else {
			return 0;
		}
	}

	public double getLeftXboxTrigger() {
		return (copilotController.getRawAxis(RobotMap.LeftXboxTrigger));
	}

	public double getRightXboxTrigger() {
		return (copilotController.getRawAxis(RobotMap.RightXboxTrigger));
	}

	public boolean getRightXboxStickButton() {
		return xboxRightStickButton.get();
	}

	public boolean getLeftXboxStickButton() {
		return xboxLeftStickButton.get();
	}

}
