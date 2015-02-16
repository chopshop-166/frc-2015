package org.usfirst.frc.team166.robot;

public class Utility {
	static final double axisZeroThreshold = 1E-6;

	public static boolean isAxisZero(double axisValue) {
		if ((axisValue < axisZeroThreshold) && (axisValue > -axisZeroThreshold)) {
			return true;
		} else {
			return false;
		}
	}
}
