/**
 * CTREpositionCloseLoop-AMedit to test single motor rot. position
 * PID tuning, uses talonSRX motor+gearbox; could config arm angle, 
 * rotating spool to lift, climb ... any mechanism needing position
 * control of rotating wheel.
 */

/**
 * Description:
 * PositionClosedLoop example demonstrates Position closed-loop servo.
 * Select the correct feedback sensor using configSelectedFeedbackSensor().
 * Use Percent Output Mode (w/ button 4 & Joystick) to confirm talon going 
 * forward (Green LED on Talon) when the position sensor is sending
 * positive numbers. If not, change the boolean value in setSensorPhase().
 * 
 * current values for MS joystick: #3-8 button for servo and Z axis for
 * manual, also work w/ gamepad;
 * CIM+talonSRX+spool bench setup has ~1550 tick/wheel rotation

 * all PID param refined in Phoenix Tuner then added to Constants.j
 * -- seem to work for light and heavy load.
 *  
 * Controls:
 * Button #3 When pressed rezero position wherever it is and set target to 0;
 * Button #4 When held, start and run Percent Output w/ Joytick Z-Axis. 
 * Button 5,6,7,8 go to preset coded position on single press.
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

public class Robot extends TimedRobot {

	WPI_TalonSRX _talon = new WPI_TalonSRX(11);
	Joystick _joystk = new Joystick(0);

	/** Used to create string to print data */
	StringBuilder _sb = new StringBuilder();
	int _loops = 0; // refresh data every loop, print Q 60 loop

	/** Track button state for single press event */
	boolean button3;
	boolean button4;
	boolean button5;
	boolean button6;
	boolean button7;
	boolean button8;

	boolean _lastButton3 = false;
	boolean _lastButton4 = false;
	boolean _lastButton5 = false;
	boolean _lastButton6 = false;
	boolean _lastButton7 = false;
	boolean _lastButton8 = false;

	/** Saves the current targeted position */
	double targetPositionRotations;

	public void robotInit() {
		/* Factory Default Talon */
		_talon.configFactoryDefault();

		/*
		 * Set based on what controller direction you want forward.
		 * according to CTRE sensor phase follows motor inversion setting
		 */
		_talon.setInverted(true);

		/* Config the sensor used for Primary PID and motor direction */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
				0, 30);

		/*
		 * Ensure sensor is positive when actuator motion is positive
		 * w/ CIM+pulley setup this keeps CW rot (lift load) positive
		 */
		_talon.setSensorPhase(true);

		/* Config the peak and nominal outputs, 1.0 means full 12v */
		_talon.configNominalOutputForward(0, 30);
		_talon.configNominalOutputReverse(0, 30);
		_talon.configPeakOutputForward(0.3, 30);
		_talon.configPeakOutputReverse(-0.3, 30);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		_talon.configAllowableClosedloopError(0, 5, 30);

		/*
		 * Configs Position Closed Loop gains in slot0, typically kF stays zero.
		 * (int slot, double value, int timeout)
		 */
		_talon.config_kF(0, Constants.kGains.kF, 30);
		_talon.config_kP(0, Constants.kGains.kP, 30);
		_talon.config_kI(0, Constants.kGains.kI, 30);
		_talon.config_kD(0, Constants.kGains.kD, 30);
		_talon.config_IntegralZone(0, Constants.kGains.kIzone, 150);

		/**
		 * Get the 0-360 degree value of the MagEncoder's absolute
		 * position, and initially set the relative sensor to match, why
		 * not do the opposite?
		 */
		// int absolutePosition = _talon.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		// absolutePosition &= 0xFFF;
		// if (Constants.kSensorPhase) { absolutePosition *= -1; }
		// if (Constants.kMotorInvert) { absolutePosition *= -1; }

		/* Set the quadrature (relative) sensor to match absolute--mag only */
		// _talon.setSelectedSensorPosition(absolutePosition, 0, 30);
	} // end robotInit

	// This function is called periodically during operator control
	// -- not clear why not put all its code into teleoPeriodic
	public void teleopPeriodic() {
		commonLoop();
	}

	void commonLoop() {
		// joystk input for manual control, Z is rt. stick on gamepad
		double leftZstick = _joystk.getZ();

		// L-hornButton rezeros sensor
		button3 = _joystk.getRawButton(3);
		// R-hornButton --> enable joystick(z) to manually control motor
		button4 = _joystk.getRawButton(4);
		// go to preset spool position, 5 --> up N tick
		button5 = _joystk.getRawButton(5);

		// button 6 --> raise to max ht (safe from 0 pos only)
		button6 = _joystk.getRawButton(6);
		// go to preset spool position, 7 --> down N tick
		boolean button7 = _joystk.getRawButton(7);
		// button 8 --> drop to lowest, only safe from top if not rezeroed
		button8 = _joystk.getRawButton(8);

		/* Deadband stick output, overriding default in motor config */
		if (Math.abs(leftZstick) < 0.04) {
			leftZstick = 0;
			/* Within 4% of zero */
		}
		// gather data and print to Console

		/* Prepare line to print */
		_sb.append("\tVout: ");
		/* Cast to int to remove decimal value */
		_sb.append((int) (_talon.getMotorOutputPercent() * 100));
		_sb.append("%  "); // Percent

		_sb.append("\tPos: ");
		_sb.append(_talon.getSelectedSensorPosition(0));
		_sb.append("u"); // Native encoder units

		/**
		 * orig ctre code used:if (!_lastButton1 && button1)
		 * When button 1 is pressed, do Position Closed Loop to
		 * target position in native encoder rotations
		 */

		// one press should stop motion & void last pos cmd on re-enabling

		if (button3) { // [pos,indx,timeout]
			_talon.setSelectedSensorPosition(0, 0, 30);
			targetPositionRotations = 0;
			_talon.set(ControlMode.Position, targetPositionRotations);
			_sb.append("\tButton3 rezeroed: ");
			_sb.append(_talon.getSelectedSensorPosition(0));
			_sb.append("u"); // Native units
		}
		/*
		 * if button 4 is held, adjust position w/ stick's Z axis;
		 */
		if (button4) {
			// if (Math.abs(leftZstick) < 0.02) {
			// _talon.setSelectedSensorPosition(0, 0, 30);
			// _sb.append(_talon.getSelectedSensorPosition(0));
			// _sb.append("u"); // Native units
			// }
			_talon.set(ControlMode.PercentOutput, leftZstick * 0.2);
		}
		// only want 1 activation / press for next 4 button
		// * rezero wasn't helpful for button positioning
		// if() condx only true on new button press @start or after
		// release; not clear how/why it runs to complete .set cmd

		if (!_lastButton5 && button5) {
			double now = (_talon.getSelectedSensorPosition());
			targetPositionRotations = now + 1500;
			_talon.set(ControlMode.Position, targetPositionRotations);
		}

		if (!_lastButton6 && button6) { // raise fully
			// only safe from zero tick = fully down
			targetPositionRotations = 8200;
			_talon.set(ControlMode.Position, targetPositionRotations);
		}

		// only want 1 activation/press: lower N tick
		if (!_lastButton7 && button7) {
			double now = (_talon.getSelectedSensorPosition());
			targetPositionRotations = now - 1500;
			_talon.set(ControlMode.Position, targetPositionRotations);
		}

		if (!_lastButton8 && button8) {
			// only safe if fully up and unzeroed
			targetPositionRotations = 0;
			_talon.set(ControlMode.Position, targetPositionRotations);
		}

		/* If Talon doing position closed-loop, gather error and target */
		if (_talon.getControlMode() == ControlMode.Position) {
			_sb.append("\tError: ");
			_sb.append(_talon.getClosedLoopError(0));
			_sb.append("u"); // Native Units (rot?)

			_sb.append("\tTarg:");
			_sb.append(targetPositionRotations);
			_sb.append("u"); /// Native Units
		}

		/**
		 * Print every N loops, printing too much too fast is bad
		 * for performance.
		 */
		if (++_loops >= 60) {
			_loops = 0;
			System.out.println(_sb.toString());
		}

		/* Empty string for fresh data next loop */
		_sb.setLength(0);

		/* Save button state for on-press detection */
		_lastButton3 = button3;
		_lastButton4 = button4;
		_lastButton5 = button5;
		_lastButton6 = button6;
		_lastButton7 = button7;
		_lastButton8 = button8;

	} // end commonLoop()

	// @Override // used to debug problem; not normally needed
	// public void disabledInit() {
	// _talon.setSelectedSensorPosition(0);
	// targetPositionRotations = 0;
	// _talon.set(ControlMode.Position, targetPositionRotations);
	// button5 = false;
	// button6 = false;
	// button7 = false;
	// button8 = false;

	// _loops = 0;
	// }

	// @Override // print pos setting button value
	// public void disabledPeriodic() {

	// _sb.append("\tButt5: ");
	// _sb.append(button5);
	// _sb.append(" ");

	// _sb.append("\tButt6: ");
	// _sb.append(button6);
	// _sb.append(" ");

	// _sb.append("\tButt7: ");
	// _sb.append(button7);
	// _sb.append("\n");

	// _sb.append("\tButt8: ");
	// _sb.append(button8);
	// _sb.append(" ");

	// _sb.append("\tPos: ");
	// _sb.append(_talon.getSelectedSensorPosition());
	// _sb.append("u "); // Native encod Units

	// _sb.append("\tTarg:");
	// _sb.append(targetPositionRotations);
	// _sb.append("u\n"); // Native encod Units

	// if (++_loops >= 60) {
	// _loops = 0;
	// System.out.println(_sb.toString());

	// } // end print
	// // purge for next disabled loop to fill string
	// _sb.setLength(0);
	// } // end disablePeriod
} // end Robot.j