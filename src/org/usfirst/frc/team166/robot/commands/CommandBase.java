package org.usfirst.frc.team166.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team166.robot.OI;
import org.usfirst.frc.team166.robot.subsystems.Drive;

/**
 * This is the base for all commands that will be implemented. All subsystems instances will be contained in this class.
 */
public abstract class CommandBase extends Command {

	protected static OI oi;

	protected static final Drive drive = new Drive();

	public CommandBase() {
		super();
	}

	public CommandBase(String name) {
		super(name);
	}

	public static void init() {
		// This MUST be here. If the OI creates Commands (which it very likely
		// will), constructing it during the construction of CommandBase (from
		// which commands extend), subsystems are not guaranteed to be yet.
		// Thus, their requires() statements may grab null pointers. Bad news.
		// Don't move it.
		oi = new OI();

		updateSmartDashboardCommands();
	}

	public static void updateSmartDashboardCommands() {
		SmartDashboard.putData(drive);
	}
}
