package org.usfirst.frc.team166.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team166.robot.commands.CommandBase;
import org.usfirst.frc.team166.robot.subsystems.Claw;
import org.usfirst.frc.team166.robot.subsystems.Drive;
import org.usfirst.frc.team166.robot.subsystems.Lift;
import org.usfirst.frc.team166.robot.subsystems.Wing;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {

	public static OI oi;
	public static final Wing leftWing = new Wing(null);// this null is gonna be a solenoid
	public static final Wing rightWing = new Wing(null);// see above
	public static final Drive drive = new Drive();// Those two juniors are working on this, so I will let them make the
													// parameter
	public static final Lift toteLift = new Lift(null);// this null is gonna be a motor
	public static final Lift rcLift = new Lift(null);// see above
	public static final Claw claw = new Claw(null);// This is gonna be a solenoid
	Command autonomousCommand;

	/**
	 * This function is run when the robot is first started up and should be used for any initialization code.
	 */
	@Override
	public void robotInit() {

		// Initialize all subsystems and the OI
		CommandBase.init();

		// instantiate the command used for the autonomous period
		autonomousCommand = null;
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		CommandBase.updateSmartDashboardCommands();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called when the disabled button is hit. You can use it to reset subsystems before shutting down.
	 */
	@Override
	public void disabledInit() {

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		CommandBase.updateSmartDashboardCommands();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
