package org.usfirst.frc.team166.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team166.robot.commands.Autonomous;
import org.usfirst.frc.team166.robot.commands.DetermineLiftCollision;
import org.usfirst.frc.team166.robot.subsystems.Claw;
import org.usfirst.frc.team166.robot.subsystems.Drive;
import org.usfirst.frc.team166.robot.subsystems.Lift;
import org.usfirst.frc.team166.robot.subsystems.LimitSwitchLift;
import org.usfirst.frc.team166.robot.subsystems.Wing;
import org.usfirst.frc.team166.robot.triggers.LiftTrigger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {

	public static OI oi;
	public static final Wing leftWing = new Wing(RobotMap.LeftWingSolenoid);
	public static final Wing rightWing = new Wing(RobotMap.RightWingSolenoid);
	public static final Drive drive = new Drive();
	public static final Lift toteLift = new Lift(RobotMap.ToteLiftMotorPwm, RobotMap.ToteLiftBrakeSolenoid,
			RobotMap.ToteEncoderA, RobotMap.ToteEncoderB, RobotMap.BotLiftLimit, Lift.LimitBoundary.Bottom);
	public static final LimitSwitchLift rcLift = new LimitSwitchLift(RobotMap.RCLiftMotorPwm,
			RobotMap.RCLiftBrakeSolenoid, RobotMap.RCEncoderA, RobotMap.RCEncoderB, RobotMap.TopLiftLimit,
			Lift.LimitBoundary.Top);
	public static final Claw claw = new Claw(null);
	private final LiftTrigger liftTrigger = new LiftTrigger();
	private Command autonomousCommand;

	/**
	 * This function is run when the robot is first started up and should be used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// This MUST be here. If the OI creates Commands (which it very likely
		// will), constructing it during the construction of CommandBase (from
		// which commands extend), subsystems are not guaranteed to be yet.
		// Thus, their requires() statements may grab null pointers. Bad news.
		// Don't move it.
		oi = new OI();

		// instantiate the command used for the autonomous period
		autonomousCommand = new Autonomous();

		// Add the command for handling when lifts collide once it's implemented
		liftTrigger.whenActive(new DetermineLiftCollision());

		// Set the drive subsystem PID controller constants from preferences
		drive.setPIDConstants();
		toteLift.liftPIDInit("Tote lift", "Tote Lift PID");
		rcLift.liftPIDInit("RC lift", "RC Lift PID");
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
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called when the disabled button is hit. You can use it to reset subsystems before shutting down.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
