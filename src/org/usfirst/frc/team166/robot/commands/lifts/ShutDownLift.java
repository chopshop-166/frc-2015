package org.usfirst.frc.team166.robot.commands.lifts;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

import org.usfirst.frc.team166.robot.subsystems.Lift;

/**
 *
 */
public class ShutDownLift extends CommandGroup {

	public ShutDownLift(Lift m_lift) {

		this.setInterruptible(false);

		addSequential(new StopLift(m_lift));
		addSequential(new WaitCommand(1));

	}
}
