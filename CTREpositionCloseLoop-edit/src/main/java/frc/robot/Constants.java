/**  ~/VSpace/CTRExampleFRCode/PositionClosedLoop/src/main/java/frc/robot/Constants
 * Simple class containing constants -- not used in edited test, most totally
 * default, unchanging, so just wasted text using this class
 */
package frc.robot;

public class Constants {
	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX supports multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 30;

	/* Choose so that Talon does not report sensor out of phase */
	public static boolean kSensorPhase = false;

	/**
	 * affects direction read from absolute to relative encoder,
	 *  does not affect motor invert itself. no use in current code
	 */
	public static boolean kMotorInvert = false;

	/**
	 * param used in Position Closed Loop, entered here, then
	 * instanced in Gains(kp, ki, kd, kf, izone, peak output) class;
	 * values of Gains instance used in call from Robot.j
	 * values here found via testing in Phoenix Tuner
	 */
	static final Gains kGains = new Gains(0.5, 0.0008, 10.0, 0.0, 300, 1.0);
}
