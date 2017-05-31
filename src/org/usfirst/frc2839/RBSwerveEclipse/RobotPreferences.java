package org.usfirst.frc2839.RBSwerveEclipse;

import edu.wpi.first.wpilibj.Preferences;

public class RobotPreferences {

	public static double LFDP() {
		return Preferences.getInstance().getDouble("LFDP", .003);
	}

	public static double LFDI() {
		return Preferences.getInstance().getDouble("LFDI", 0.0);
	}

	public static double LFDD() {
		return Preferences.getInstance().getDouble("LFDD", 0.0);
	}
	public static double LFDF() {
		return Preferences.getInstance().getDouble("LFDF", 0.0);
	}

}
