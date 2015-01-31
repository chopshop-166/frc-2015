package org.usfirst.frc.team166.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team166.robot.commands.Autonomous;
import org.usfirst.frc.team166.robot.commands.CancelDriveCommand;
import org.usfirst.frc.team166.robot.commands.DriveDirection;
import org.usfirst.frc.team166.robot.commands.LiftWings;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands and command groups
 * that allow control of the robot.
 */
public class OI {

	private final Joystick driveJoystick = new Joystick(RobotMap.DriveJoystick);

	public OI() {

		JoystickButton button1 = new JoystickButton(driveJoystick, 1);
		JoystickButton button2 = new JoystickButton(driveJoystick, 2);
		JoystickButton button3 = new JoystickButton(driveJoystick, 3);
		JoystickButton button4 = new JoystickButton(driveJoystick, 4);
		button2.whenPressed(new CancelDriveCommand());
		button3.whileHeld(new DriveDirection(90));
		button4.whenPressed(new Autonomous());
		SmartDashboard.putData("LiftWings", new LiftWings());
	}

	public Joystick getDriveJoystick() {
		return driveJoystick;
	}
}
