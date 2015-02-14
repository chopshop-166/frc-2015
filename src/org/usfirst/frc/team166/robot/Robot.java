package org.usfirst.frc.team166.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team166.robot.commands.autonomous.Autonomous;
import org.usfirst.frc.team166.robot.commands.lifts.DetermineLiftCollision;
import org.usfirst.frc.team166.robot.commands.lifts.ShutDownLift;
import org.usfirst.frc.team166.robot.subsystems.Claw;
import org.usfirst.frc.team166.robot.subsystems.Drive;
import org.usfirst.frc.team166.robot.subsystems.Lift;
import org.usfirst.frc.team166.robot.subsystems.LimitSwitchLift;
import org.usfirst.frc.team166.robot.subsystems.Wing;
import org.usfirst.frc.team166.robot.triggers.CarriageTrigger;
import org.usfirst.frc.team166.robot.triggers.RCLiftStalled;
import org.usfirst.frc.team166.robot.triggers.ToteLiftStalled;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {

	public static OI oi;
	public static final PowerDistributionPanel pdBoard = new PowerDistributionPanel();
	public static final Wing leftWing = new Wing("Left Wing", RobotMap.solenoid.LeftWingForward,
			RobotMap.solenoid.LeftWingReverse);
	public static final Wing rightWing = new Wing("Right Wing", RobotMap.solenoid.RightWingForward,
			RobotMap.solenoid.RightWingReverse);
	public static final Drive drive = new Drive();
	public static final Lift toteLift = new Lift(RobotMap.Pwm.ToteLiftMotor, RobotMap.solenoid.ToteLiftBrakeForward,
			RobotMap.solenoid.ToteLiftBrakeReverse, RobotMap.Encoders.ToteLiftA, RobotMap.Encoders.ToteLiftB,
			RobotMap.Switch.LiftLowerLimit, "Tote");
	public static final LimitSwitchLift rcLift = new LimitSwitchLift(RobotMap.Pwm.RCLiftMotor,
			RobotMap.solenoid.RCLiftBrakeForward, RobotMap.solenoid.RCLiftBrakeReverse, RobotMap.Encoders.RCLiftA,
			RobotMap.Encoders.RCLiftB, RobotMap.Switch.LiftUpperLimit, "RC");
	public static final Claw claw = new Claw();

	// Triggers
	private final ToteLiftStalled toteLiftStalled = new ToteLiftStalled();
	private final RCLiftStalled rcLiftStalled = new RCLiftStalled();
	private final CarriageTrigger carriageTrigger = new CarriageTrigger();
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

		// Connect triggers to commands
		carriageTrigger.whenActive(new DetermineLiftCollision());
		toteLiftStalled.whenActive(new ShutDownLift(Robot.toteLift));
		rcLiftStalled.whenActive(new ShutDownLift(Robot.rcLift));

		// Subsystem initialization
		drive.setPIDConstants();
		toteLift.initLift();
		rcLift.initLift();

		// Set the claw setState
		claw.setState();

		updateSubsystemSmartdashboard();

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
		updateSubsystemSmartdashboard();
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
		updateSubsystemSmartdashboard();
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

	private void updateSubsystemSmartdashboard() {
		SmartDashboard.putData(leftWing);
		SmartDashboard.putData(rightWing);
		SmartDashboard.putData(drive);
		SmartDashboard.putData(toteLift);
		SmartDashboard.putData(rcLift);
		SmartDashboard.putData(claw);
	}
}
