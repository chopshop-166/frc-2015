package org.usfirst.frc.team166.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team166.robot.commands.autonomous.NothingAutonomous;
import org.usfirst.frc.team166.robot.commands.autonomous.ToteAndRCAutonomous;
import org.usfirst.frc.team166.robot.commands.autonomous.ToteOrRCAutonomous;
import org.usfirst.frc.team166.robot.commands.claw.ToggleClaw;
import org.usfirst.frc.team166.robot.commands.lifts.DetermineLiftCollision;
import org.usfirst.frc.team166.robot.commands.lifts.LowerRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.LowerToteLift;
import org.usfirst.frc.team166.robot.commands.lifts.RaiseRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.RaiseToteLift;
import org.usfirst.frc.team166.robot.commands.lifts.ShutDownRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.ShutDownToteLift;
import org.usfirst.frc.team166.robot.commands.lifts.SlowLowerRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.SlowLowerToteLift;
import org.usfirst.frc.team166.robot.commands.lifts.SlowRaiseRCLift;
import org.usfirst.frc.team166.robot.commands.lifts.SlowRaiseToteLift;
import org.usfirst.frc.team166.robot.subsystems.Claw;
import org.usfirst.frc.team166.robot.subsystems.Drive;
import org.usfirst.frc.team166.robot.subsystems.JankShank;
import org.usfirst.frc.team166.robot.subsystems.Lift;
import org.usfirst.frc.team166.robot.subsystems.LimitSwitchLift;
import org.usfirst.frc.team166.robot.subsystems.Wing;
import org.usfirst.frc.team166.robot.triggers.ActuateClawTrig;
import org.usfirst.frc.team166.robot.triggers.CarriageTrigger;
import org.usfirst.frc.team166.robot.triggers.RCLiftDownTrig;
import org.usfirst.frc.team166.robot.triggers.RCLiftStalled;
import org.usfirst.frc.team166.robot.triggers.RCLiftUpTrig;
import org.usfirst.frc.team166.robot.triggers.SlowRCLiftDownTrig;
import org.usfirst.frc.team166.robot.triggers.SlowRCLiftUpTrig;
import org.usfirst.frc.team166.robot.triggers.SlowToteLiftDownTrig;
import org.usfirst.frc.team166.robot.triggers.SlowToteLiftUpTrig;
import org.usfirst.frc.team166.robot.triggers.ToteLiftDownTrig;
import org.usfirst.frc.team166.robot.triggers.ToteLiftStalled;
import org.usfirst.frc.team166.robot.triggers.ToteLiftUpTrig;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {

	public static OI oi;
	// public static SendableChooser autoChooser;
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
	public static final JankShank jankShank = new JankShank();

	// Triggers
	private final ToteLiftStalled toteLiftStalled = new ToteLiftStalled();
	private final RCLiftStalled rcLiftStalled = new RCLiftStalled();
	private final CarriageTrigger carriageTrigger = new CarriageTrigger();

	private final RCLiftUpTrig rcLiftUp = new RCLiftUpTrig();
	private final RCLiftDownTrig rcLiftDown = new RCLiftDownTrig();
	private final ToteLiftUpTrig toteLiftUp = new ToteLiftUpTrig();
	private final ToteLiftDownTrig toteLiftDown = new ToteLiftDownTrig();

	private final SlowRCLiftUpTrig slowRCLiftUp = new SlowRCLiftUpTrig();
	private final SlowRCLiftDownTrig slowRCLiftDown = new SlowRCLiftDownTrig();
	private final SlowToteLiftUpTrig slowToteLiftUp = new SlowToteLiftUpTrig();
	private final SlowToteLiftDownTrig slowToteLiftDown = new SlowToteLiftDownTrig();
	private final ActuateClawTrig actuateClaw = new ActuateClawTrig();

	// Auto Chooser
	private SendableChooser autoChooser = new SendableChooser();

	private Command autonomousCommand;

	public static double toteCount;

	/**
	 * This function is run when the robot is first started up and should be used for any initialization code.
	 */
	@Override
	public void robotInit() {
		autoChooser.addDefault("Tote AND RC", new ToteAndRCAutonomous());
		autoChooser.addObject("Tote OR RC", new ToteOrRCAutonomous());
		autoChooser.addObject("Paper Weight", new NothingAutonomous());

		SmartDashboard.putData("Autonomous", autoChooser);

		toteCount = 0;
		// This MUST be here. If the OI creates Commands (which it very likely
		// will), constructing it during the construction of CommandBase (from
		// which commands extend), subsystems are not guaranteed to be yet.
		// Thus, their requires() statements may grab null pointers. Bad news.
		// Don't move it.

		// instantiate the command used for the autonomous period
		oi = new OI();

		// Add options and default for autonomous chooser

		// autoChooser.addObject("Tote and RC auto", new RCToteAutonomous());
		// autoChooser.addObject("Retrieve RCs", new StepRCAutonomous());
		// autoChooser.addDefault("Tote and RC auto", new RCToteAutonomous());
		// autonomousCommand = (Command) autoChooser.getSelected();
		// autonomousCommand = (new AutoJank());
		// autonomousCommand = (new NothingAutonomous());
		// autonomousCommand = (new StackTwoTotes());
		autonomousCommand = (new ToteOrRCAutonomous());
		// autonomousCommand = null;
		// Connect triggers to commands
		carriageTrigger.whileActive(new DetermineLiftCollision());
		toteLiftStalled.whenActive(new ShutDownToteLift());
		rcLiftStalled.whenActive(new ShutDownRCLift());

		// Copilot Control Triggers
		rcLiftUp.whileActive(new RaiseRCLift());
		rcLiftDown.whileActive(new LowerRCLift());
		toteLiftUp.whileActive(new RaiseToteLift());
		toteLiftDown.whileActive(new LowerToteLift());

		slowRCLiftUp.whileActive(new SlowRaiseRCLift());
		slowRCLiftDown.whileActive(new SlowLowerRCLift());
		slowToteLiftUp.whileActive(new SlowRaiseToteLift());
		slowToteLiftDown.whileActive(new SlowLowerToteLift());
		actuateClaw.whenActive(new ToggleClaw());

		// Subsystem initialization
		drive.setPIDConstants();
		toteLift.initLift();
		rcLift.initLift();
		claw.setState();

		updateSubsystemSmartdashboard();

	}

	@Override
	public void autonomousInit() {
		// autonomousCommand = (Command) autoChooser.getSelected();

		if (autonomousCommand != null)
			autonomousCommand.start();
		Robot.drive.resetIntegral();
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
		Robot.drive.resetIntegral();
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
