/**
 * CTREpositionCloseLoop-AMedit to test single motor positioning
 * PID tuning, w/ talonSRX motor+gearbox setting arm angle
 */

/**
 * Description:
 * The PositionClosedLoop example demonstrates the Position closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station
 * current values for MS joystick, its 3 = button 1, its 4 = button 2 in code
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor().
 * Use Percent Output Mode (Holding  and using Left Joystick) to confirm talon is driving 
 * forward (Green LED on Talon/Victor) when the position sensor is moving in the positive 
 * direction. If this is not the case, flip the boolean input in setSensorPhase().
 * 
 * Controls:
 * Button 1:(#3) When pressed, start and run Position Closed Loop on Talon/Victor
 * Button 2:(4) When held, start and run Percent Output
 * Left Joytick Y-Axis:
 * 	+ Position Closed Loop: Servo Talon forward and reverse [-10, 10] rotations
 * 	+ Percent Ouput: Throttle Talon forward and reverse
 * 
 * Gains for Position Closed Loop will be adjusted in PIDset.java
 * if you can find it; other params from Constant hardcoded for clarity
 * all sim stuff commented out, lib deleted
 * 
 * Supported Version:
 * - Talon SRX: 4.00
 * - Victor SPX: 4.00
 * - Pigeon IMU: 4.00
 * - CANifier: 4.00
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

// import frc.robot.sim.PhysicsSim;

public class Robot extends TimedRobot {
	/** Hardware */
	WPI_TalonSRX _talon = new WPI_TalonSRX(11);
	Joystick _joystk = new Joystick(0);

	/** Used to create string to print data */
	StringBuilder _sb = new StringBuilder();
	int _loops = 0;

	/** Track button state for single press event */
	boolean _lastButton1 = false;

	/** Save the target position */
	double targetPositionRotations;

	// public void simulationInit() {
	// PhysicsSim.getInstance().addTalonSRX(_talon, 0.75, 4000,
	// Constants.kSensorPhase);
	// }
	// public void simulationPeriodic() {
	// PhysicsSim.getInstance().run();

	public void robotInit() {
		/* Factory Default all hardware */
		_talon.configFactoryDefault();

		/**
		 * Set based on what direction you want forward.
		 * according to CTRE sensor phase follows motor inversion setting
		 */
		_talon.setInverted(false);

		/* Config the sensor used for Primary PID and sensor direction */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
				0,
				30);

		/* Ensure sensor is positive when actuator motion is positive */
		_talon.setSensorPhase(false);

		/* Config the peak and nominal outputs, 12V means full */
		_talon.configNominalOutputForward(0, 30);
		_talon.configNominalOutputReverse(0, 30);
		_talon.configPeakOutputForward(0.5, 30);
		_talon.configPeakOutputReverse(-0.5, 30);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		// _talon.configAllowableClosedloopError(0, 400, 30);

		/* Config Position Closed Loop gains in slot0, typically kF stays zero.
		(int slot, double value, int timeout) 
		*/
		_talon.config_kF(0, Constants.kGains.kF, 30);
		_talon.config_kP(0, Constants.kGains.kP, 30);
		_talon.config_kI(0, Constants.kGains.kI, 30);
		_talon.config_kD(0, Constants.kGains.kD, 30);

		/**
		 * Get the 0-360 degree value of the MagEncoder's absolute
		 * position, and initially set the relative sensor to match, why?
		 * not do the opposite.
		 */
		// int absolutePosition = _talon.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		// absolutePosition &= 0xFFF;
		// if (Constants.kSensorPhase) { absolutePosition *= -1; }
		// if (Constants.kMotorInvert) { absolutePosition *= -1; }

		/* Set the quadrature (relative) sensor to match absolute--mag only */
		// _talon.setSelectedSensorPosition(absolutePosition, 0, 30);
	}   // end robotInit

	void commonLoop() {
		/* joystk input */
		double leftYstick = _joystk.getY();
		double joySlide = _joystk.getRawAxis(3);
		// l-hornButton triggers position mode, value from slider 0-1
		boolean button1 = _joystk.getRawButton(3); 
		// r-hornButton triggers rezero then joystick control +/-
		boolean button2 = _joystk.getRawButton(4); 

		/* Get Talon/Victor's current output percentage */
		double motorOutput = _talon.getMotorOutputPercent();

		/* Deadband stick output, overriding defaults in motor config */
		if (Math.abs(leftYstick) < 0.05) {
			/* Within 5% of zero */
			leftYstick = 0;
		}
		if (Math.abs(joySlide) < 0.05) {
			/* Within 5% of zero */
			joySlide = 0;
		}

		/* Prepare line to print */
		_sb.append("\tVout:");
		/* Cast to int to remove decimal places */
		_sb.append((int) (motorOutput * 100));
		_sb.append("%"); // Percent

		_sb.append("\tpos:");
		_sb.append(_talon.getSelectedSensorPosition(0));
		_sb.append("u"); // Native units

		/**
		 * When button 1 is pressed, do Position Closed Loop to
		 * joySlide position x10, [-10, 10] rotations
		 * only true on new button press @start or after previous release
		 * so not clear if it runs every loop
		 */
		if (!_lastButton1 && button1) {

			/* 10 Rotations * 4096 u/rev in 0-1 direction  w/ slide */
			targetPositionRotations = joySlide * 10.0 * 4096;
			_talon.set(ControlMode.Position, targetPositionRotations);
		}

		/*
		 * if button 2 is held, adjust position w/ stick's Y axis
		 * but on brief press if stick < deadband just zero encoder
		 */
		if (button2) {
			if (Math.abs(leftYstick) < 0.02) {
				_talon.setSelectedSensorPosition(0, 0, 30);
				_sb.append("\trezero by button:");
				_sb.append(_talon.getSelectedSensorPosition(0));
				_sb.append("u"); // Native units
			}

			_talon.set(ControlMode.PercentOutput, leftYstick);
		}

		/* If Talon doing position closed-loop, gather error and target # */
		if (_talon.getControlMode() == ControlMode.Position) {
			/* append more info to print when in posit mode. */
			_sb.append("\terro:");
			_sb.append(_talon.getClosedLoopError(0));
			_sb.append("u"); // Native Units (rot?)

			_sb.append("\ttarg:");
			_sb.append(targetPositionRotations);
			_sb.append("u"); /// Native Units
		}

		/**
		 * Print every ten loops, printing too much too fast is generally bad
		 * for performance.
		 */
		if (++_loops >= 20) {
			_loops = 0;
			System.out.println(_sb.toString());
		}

		/* Reset built string for next loop */
		_sb.setLength(0);

		/* Save button state for on press detect */
		_lastButton1 = button1;
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		commonLoop();
	}
}
