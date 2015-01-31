package org.usfirst.frc.team166.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team166.robot.commands.Autonomous;
import org.usfirst.frc.team166.robot.commands.CancelDriveCommand;
import org.usfirst.frc.team166.robot.commands.DriveDirection;
import org.usfirst.frc.team166.robot.commands.LiftWings;
import org.usfirst.frc.team166.robot.commands.LowerWings;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands and command groups
 * that allow control of the robot.
 */
public class OI {

	private final Joystick driveJoystick = new Joystick(RobotMap.DriveJoystick);
	private final Joystick toteJoystick = new Joystick(RobotMap.ToteJoystick);
	private final Joystick rcJoystick = new Joystick(RobotMap.RCJoystick);

	public OI() {

		JoystickButton button1 = new JoystickButton(driveJoystick, 1);
		JoystickButton button2 = new JoystickButton(driveJoystick, 2);
		JoystickButton button3 = new JoystickButton(driveJoystick, 3);
		JoystickButton button4 = new JoystickButton(driveJoystick, 4);
		button2.whenPressed(new CancelDriveCommand());
		button3.whileHeld(new DriveDirection(90));
		button4.whenPressed(new Autonomous());

		SmartDashboard.putData("LiftWings", new LiftWings());
		SmartDashboard.putData("LowerWings", new LowerWings());

	}

	public Joystick getDriveJoystick() {
		return driveJoystick;
	}

	public Joystick getToteJoystick() {
		return toteJoystick;
	}

	public Joystick getRCJoystick() {
		return rcJoystick;
	}

}
