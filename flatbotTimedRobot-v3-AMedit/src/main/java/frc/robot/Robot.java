/* flatbotTimedRobot-v2-AMedit */

/* originally SunCode#2timedRobot, simple flat framework, edited 
2211+ --> new vers. flatbotTRv1: single joystick, TalonSRX x4, 
CA drive, auto from encoder distance, no PID, send encoder values 
to smart dashboard. 221225+, new v 2, add PID control drive in Auto 
periodic, button pos.control of drive in teleop, straight drive.
221229+ new v 3 add Auto option via chooser, sequential Cmd, rot.,
reverse path, ? need Cmd object to do sequential? keep flat format?
*/
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

// import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation.
 */
public class Robot extends TimedRobot {
  // declare / instance hardware, I/O device, motor enabler class,
  // unit conversion, constant

  // actuators
  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(3);
  private WPI_TalonSRX leftSlave = new WPI_TalonSRX(4);

  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(1);
  private WPI_TalonSRX rightSlave = new WPI_TalonSRX(2);

  // private WPIleftMasterSRX/ armMotor = new WPIleftMasterSRX(5);
  // private WPIleftMasterSRX armSlave = new WPIleftMasterSRX(3);
  // private WPIleftMasterSRX rollerMotor = new WPIleftMasterSRX(4);
  // // private Compressor compressor = new Compressor(null);
  // private DoubleSolenoid hatchIntake = new
  // DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1); // PCM port 0, 1

  // drive mode class instanced using 2 motor instanced above
  // -- gives you arcadeDrive() et al methods
  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  // joysticks
  private Joystick driverJoystick = new Joystick(0);
  // private Joystick operatorJoystick = new Joystick(1);

  /** creates string to print data to console */
  StringBuilder _sb = new StringBuilder();
  int _loops = 0; // refresh data every loop, print Q50 loop

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

  // param for auto drive to position, set below ln.
  double targetDriveFt = 4.0;
  int targetRotation;

  // unit conversion for flatbot, 1 wheel rot, 18.7in = 10700 tick
  // 4.5 ft = ~32000
  private final double kDriveFt2Tick = (10700 * 12) / (6 * Math.PI);
  // private final double kDriveTick2Feet = (6 * Math.PI / 12) / 10700;
  // private final double kArmTick2Deg = 360.0 / 512 * 26 / 42 * 18 /
  // 60 * 18 / 84;

  // set here after tuning in PT
  static final double kP = 0.3;
  static final double kI = 0.00015;
  static final double kD = 50.0;
  static final double kF = 0.0;
  static final double kIzone = 1200;

  @Override
  public void robotInit() { // obj settings valid here, not before.
    // CommandScheduler.getInstance().run();
    // invert motor -- usually one or other drive should be false
    leftMaster.setInverted(true);
    rightMaster.setInverted(false);
    // armMotor.setInverted(false);

    // slave setups
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    // armSlave.follow//(armMotor);

    // should follow, so maybe not needed?
    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
    // armSlave.setInverted(InvertType.FollowMaster);

    // init remote encoder wired via these TalonSRX
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
        0, 20);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
        0, 20);
    // armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
    // 0, 20);

    /*
     * Sets phase of the sensor. Use when controller forward/reverse output
     * doesn't correlate to appropriate forward/reverse reading of sensor.
     * Pick a value so that positive PercentOutput yields a positive change in
     * sensor. Should follow controller invert setting, so may not need?
     * After setting this, user can freely call SetInverted() with any value.
     */
    leftMaster.setSensorPhase(true);
    rightMaster.setSensorPhase(true);
    // armMotor.setSensorPhase(true);

    // reset encoders to zero [pos, index0-3, timeout ms.]
    leftMaster.setSelectedSensorPosition(0, 0, 20);
    rightMaster.setSelectedSensorPosition(0, 0, 20);
    // armMotor.setSelectedSensorPosition(0, 0, 20);

    // set position limit: to stop motor in soft vs. hardware
    // armMotor.configReverseSoftLimitThreshold((int) (0 / kArmTick2Deg), 20);
    // armMotor.configForwardSoftLimitThreshold((int) (175 / kArmTick2Deg), 20);
    // armMotor.configReverseSoftLimitEnable(true, 20);
    // armMotor.configForwardSoftLimitEnable(true, 20);
    // start compressor, automatic now
    // compressor.start();

    /*
     * Config peak and nominal (min) outputs, 1.0 means full 12v
     * doubt all necessary to code, if defaults usable
     */
    leftMaster.configNominalOutputForward(0, 20);
    rightMaster.configNominalOutputForward(0, 20);

    leftMaster.configNominalOutputReverse(0, 20);
    rightMaster.configNominalOutputReverse(0, 20);

    leftMaster.configPeakOutputForward(0.8, 20);
    leftMaster.configPeakOutputReverse(-0.8, 20);

    rightMaster.configPeakOutputForward(0.8, 20);
    rightMaster.configPeakOutputReverse(-0.8, 20);

    // Closed-Loop output will be neutral within this range
    leftMaster.configAllowableClosedloopError(0, 50, 20);
    rightMaster.configAllowableClosedloopError(0, 50, 20);

    // Configs Position Closed Loop gains in slot0; typically kF
    // stays zero. (int slot, double k_value, int timeout)
    leftMaster.config_kP(0, kP, 20);
    leftMaster.config_kI(0, kI, 20);
    leftMaster.config_kD(0, kD, 20);
    leftMaster.config_kF(0, kF, 20);
    leftMaster.config_IntegralZone(0, kIzone, 20);

    // dampen abrupt starts in manual mode
    leftMaster.configOpenloopRamp(0.1, 20);
    rightMaster.configOpenloopRamp(0.1, 20);
    // overrides default of 0.02 I think
    drive.setDeadband(0.04);
    // // set target # here
    // targetDriveFt = 4;
    targetRotation = (int) (targetDriveFt * kDriveFt2Tick);

  } // end robotInit

  @Override
  public void robotPeriodic() { // needs 2 be here -->
    CommandScheduler.getInstance().run();
    // SmartDashboard.putNumber("Arm Encoder Value",//
    // armMotor.getSelectedSensorPosition() * kArmTick2Deg);
    // SmartDashboard.putNumber("LeftDriveDist (ft)",
    // leftMaster.getSelectedSensorPosition() * kDriveTick2Feet);
    // SmartDashboard.putNumber("RightDriveDist (ft)",
    // rightMaster.getSelectedSensorPosition() * kDriveTick2Feet);

    /* fill array to print data to console */
    _sb.append("\tVout: ");
    /* x100 and cast to int to remove extra decimal value */
    _sb.append((int) (leftMaster.getMotorOutputPercent() * 100));
    _sb.append("%  "); // Percent

    _sb.append("\tRota: ");
    _sb.append(leftMaster.getSelectedSensorPosition(0));
    _sb.append("u  "); // Native encoder units, 10700 = 1 wheel rot.

    _sb.append("\ttargRota: ");
    _sb.append(targetRotation);
    _sb.append("\t u  ");

  } // end robotPeriodic

  @Override
  public void autonomousInit() {

    enableMotors(true); // sets all to neutral-Brake
    // reset encoders to zero [pos, index0-3, timeout]
    leftMaster.setSelectedSensorPosition(0, 0, 20);
    rightMaster.setSelectedSensorPosition(0, 0, 20);
    // armMotor.setSelectedSensorPosition(0, 0, 20);

  } // end autoInit

  @Override // no defined auto Cmd, but there could be, and called here
  public void autonomousPeriodic() {

    leftMaster.set(ControlMode.Position, targetRotation);
    rightMaster.set(ControlMode.Position, targetRotation);
    // expected to print from roboPeriod but not unless here too
    _sb.append("\tVout: ");
    /* Cast to int to remove extra decimal # */
    _sb.append((int) (leftMaster.getMotorOutputPercent() * 100));
    _sb.append("%  ");

    _sb.append("\tRota: ");
    _sb.append(leftMaster.getSelectedSensorPosition(0));
    _sb.append("u  "); // Native encoder units, 10700 = 1 wheel rot.

    _sb.append("\ttargRota: ");
    _sb.append(targetRotation);
    _sb.append("\t u  ");

    if (++_loops >= 60) {
      _loops = 0;
      System.out.println(_sb.toString());
    }
    /* Empty string for fresh data next loop */
    _sb.setLength(0);
    // needed ? -- no
    // CommandScheduler.getInstance().run();
  } // end autoPeriod

  @Override
  public void teleopInit() {
    // CommandScheduler.getInstance().run(); done by roboPeri
    enableMotors(true); // sets to Brake
  }

  @Override
  public void teleopPeriodic() {
    // CommandScheduler.getInstance().run();
    double power = -driverJoystick.getRawAxis(1); // rem: - sign
    double turn = driverJoystick.getRawAxis(0);
    // deadband set in DD class, this overrides
    if (Math.abs(power) < 0.04) {
      power = 0;
    }
    if (Math.abs(turn) < 0.04) {
      turn = 0;
    } // use stick values in DD's arcade method
    drive.arcadeDrive(power * 0.5, turn * 0.5);

    // L-hornButton rezeros sensor
    button3 = driverJoystick.getRawButton(3);
    // r-hornButton --> enable joystick(z) 2 control motor
    // button4 = driverJoystick.getRawButton(4);
    // go to preset spool position, 5
    button5 = driverJoystick.getRawButton(5);

    // now should stop motion & void last cmd on re-enable
    if (button3) { // [pos,indx,timeout]
      leftMaster.setSelectedSensorPosition(0, 0, 30);
      rightMaster.setSelectedSensorPosition(0, 0, 30);
      int targZeroRota = 0;
      leftMaster.set(ControlMode.Position, targZeroRota);
      rightMaster.set(ControlMode.Position, targZeroRota);
      _sb.append("\tButton3 rezeroed: ");
      _sb.append(leftMaster.getSelectedSensorPosition(0));
      _sb.append("u  "); // Native units
    }

    // ctre exampl: only wanted 1 press activ., used !_lastB && butt5
    // this needs constant press to drive it to target
    if (button5) {
      leftMaster.set(ControlMode.Position, -targetRotation);
      rightMaster.set(ControlMode.Position, -targetRotation);
    }
    // arm [device] control w/ stick or button, +/- PID could execute here
    // double armPower = -operatorJoystick.getRawAxis(1);
    // if (Math.abs(armPower) < 0.05) {
    // armPower = 0;
    // }
    // armPower *= 0.5;
    // armMotor.set(ControlMode.PercentOutput, armPower);

    // roller control
    // double rollerPower = 0;
    // if (operatorJoystick.getRawButton(1) == true) {
    // rollerPower = 1;
    // } else if (operatorJoystick.getRawButton(2)) {
    // rollerPower = -1;
    // }
    // rollerMotor.set(ControlMode.PercentOutput, rollerPower);

    // // hatch intake
    // if (operatorJoystick.getRawButton(3)) {// open
    // hatchIntake.set(Value.kReverse);
    // } else {
    // hatchIntake.set(Value.kForward);
    // }
    /**
     * Print every N loops, printing too much too fast is bad
     * for performance.
     */
    if (++_loops >= 60) {
      _loops = 0;
      System.out.println(_sb.toString());
    }
    /* Empty string gets fresh data next loop */
    _sb.setLength(0);

    /* Save button state for on-press detection */
    // _lastButton3 = button3;
    // _lastButton4 = button4;
    _lastButton5 = button5;

  } // end teleopPeriodic

  @Override
  public void disabledInit() {
    enableMotors(true); // keep in brake to hold pos.
  }

  private void enableMotors(boolean on) {
    NeutralMode mode;
    if (on) {
      mode = NeutralMode.Brake;
    } else {
      mode = NeutralMode.Coast;
    }
    leftMaster.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    leftSlave.setNeutralMode(mode);
    rightSlave.setNeutralMode(mode);
    // armMotor.setNeutralMode(mode);
    // armSlave.setNeutralMode(mode);
    // rollerMotor.setNeutralMode(mode);
  } // end enableMotor

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
} // end Robot.j
