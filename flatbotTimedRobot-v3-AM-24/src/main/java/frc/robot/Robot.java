/* flatbotTimedRobot-v3-AM-24 */

/* originally SunCode#2timedRobot, simple flat framework, edited 
2211+ --> new vers. flatbotTRv1: single joystick, TalonSRX x4, 
CA drive, auto from encoder distance, no PID, send encoder values 
to console. 
221225+, new v 2, add PID control drive in Auto periodic, button 
pos.control of drive in teleop, straight drive fwd/bak.

240731 post motor burn, testing function, tuning up flat format, same
single auto straight run in autoPeriod, need PID tuning for smooth move
240831 import to '24 VSC, needs '24 DS and RIO image, phenx 5+6 to run

in teleOp R side appears to go farther on straight driving than L by encod
ticks, also L < R ticks on turns. But manual wheel turn x10 reads 
almost equal (15.2 L vs 15.6 R ). In auto both sides go ~ 4 ft, end simul-
taneously. Is L motor pair weaker than R? How does it perform on ground?
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation.
 */
public class Robot extends TimedRobot {
  // declare & define class field members; no config possible here
  // instance hardware, I/O device, motor enabler class,
  // unit conversion, constant

  // actuators
  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(3);
  private WPI_TalonSRX leftSlave = new WPI_TalonSRX(4);

  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(1);
  private WPI_TalonSRX rightSlave = new WPI_TalonSRX(2);

  // private WPIleftMasterSRX/ armMotor = new WPIleftMasterSRX(5);
  // private WPIleftMasterSRX armSlave = new WPIleftMasterSRX(3);
  // private WPIleftMasterSRX rollerMotor = new WPIleftMasterSRX(4);
  // private Compressor compressor = new Compressor(null);
  // private DoubleSolenoid hatchIntake = new
  // DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1); // PCM port 0, 1

  // drive mode class instanced using 2 motor instanced above
  // -- gives you arcadeDrive() et al methods
  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  // joysticks
  private Joystick driverJoystick = new Joystick(0);
  // private Joystick operatorJoystick = new Joystick(1);

  /** creates string array to print data to console */
  StringBuilder _sb = new StringBuilder();
  int _loops = 0; // refresh data every loop, print Q 60 loop

  /** Track button state for single press event */
  boolean button1; // A on gamepad, rezero encoders
  // boolean button4;
  // boolean button5;
  // boolean button6;
  // boolean button7;
  // boolean button8;

  // boolean _lastButton3 = false;
  // boolean _lastButton4 = false;
  // boolean _lastButton5 = false;
  // boolean _lastButton6 = false;
  // boolean _lastButton7 = false;
  // boolean _lastButton8 = false;


  // unit conversion for flatbot encoder:
  // wheel diam. = 0.5 ft, 1 wheel rot= 1.571 ft; counting wheel rot x10
  // and phoenix count of ticks, we calc. 1 wheel rot = 10700 tick.
  // since gear ratio 8:1 (also encod ratio?), 1 encod. rot. = 1338 tick.
  // E4T cpr is supposed to be 360, but SRX reads all 4 transitions (?true)
  // so ideally counts 1440 cpr. Our reading 1338 could indicate slightly 
  // damaged rotor or other problem
  
  private final double kDriveFt2Tick = 10700 / (0.5 * Math.PI); // = 6811 ct./ ft
  private final double kDriveTick2Feet = (0.5 * Math.PI) / 10700;
  // private final double kArmTick2Deg = 360.0 / 512 * 26 / 42 * 18 

    // param for auto drive to position, set here so roboPerio has non-
    // null variable to print in all mode
    double targetDriveFt = 4.0;
    int targetTick = (int) (targetDriveFt * kDriveFt2Tick);

  // set here after tuning in PhenxTuner
  static final double kP = 0.15; // would smaller make gentler start? Y
  static final double kI = 0.00015;
  static final double kD = 100.0;  // made gradual motion end smoother
  static final double kF = 0.0; 
  static final double kIzone = 1200;

  @Override
  public void robotInit() { // obj param settings valid here, not before.
    // invert motor -- usually R or L drive should be false
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

    // init remote encoder x2 wired via masters' TalonSRX
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
     * After setting this, user can config motor's SetInverted() to any value
     * [and sensor will follow it?].
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
     * doubt all necessary to code, if defaults work
     * nominal means speed at Idle
     */
    leftMaster.configNominalOutputForward(0, 20);
    rightMaster.configNominalOutputForward(0, 20);

    leftMaster.configNominalOutputReverse(0, 20);
    rightMaster.configNominalOutputReverse(0, 20);

    leftMaster.configPeakOutputForward(0.5, 20);
    leftMaster.configPeakOutputReverse(-0.5, 20);

    rightMaster.configPeakOutputForward(0.5, 20);
    rightMaster.configPeakOutputReverse(-0.5, 20);

    // dampen abrupt starts in manual mode
    leftMaster.configOpenloopRamp(1.0, 20);
    rightMaster.configOpenloopRamp(1.0, 20);

    // try for less jerky auto start
    leftMaster.configClosedloopRamp(1.0);
    rightMaster.configClosedloopRamp(1.0);
    
    // Closed-Loop output will be neutral within this range
    leftMaster.configAllowableClosedloopError(0, 50, 20);
    rightMaster.configAllowableClosedloopError(0, 50, 20);


    // Configs Position Closed Loop gains from above in slot0; typically kF
    // stays zero. (int slot, double k_value, int timeout)
    leftMaster.config_kP(0, kP, 20);
    leftMaster.config_kI(0, kI, 20);
    leftMaster.config_kD(0, kD, 20);
    leftMaster.config_kF(0, kF, 20);
    leftMaster.config_IntegralZone(0, kIzone, 20);

    rightMaster.config_kP(0, kP, 20);
    rightMaster.config_kI(0, kI, 20);
    rightMaster.config_kD(0, kD, 20);
    rightMaster.config_kF(0, kF, 20);
    rightMaster.config_IntegralZone(0, kIzone, 20);

    // overrides default of 0.02 I think
    drive.setDeadband(0.04); 

  } // end robotInit

  @Override
  public void robotPeriodic() { // needs 2 .run altho we don't have any Cmd or
  // Subsys objects here. Could be needed to run Periodic method themselves?
    CommandScheduler.getInstance().run();

      // L-horn Button / game pad's A  rezeros sensor
      button1 = driverJoystick.getRawButton(1);
   
      // should stop motion & void last cmd on re-enable
      if (button1) { // [pos,indx,timeout]
        leftMaster.setSelectedSensorPosition(0, 0, 30);
        rightMaster.setSelectedSensorPosition(0, 0, 30);
        int targZeroRota = 0;
        leftMaster.set(ControlMode.Position, targZeroRota);
        rightMaster.set(ControlMode.Position, targZeroRota);
        _sb.append("\tButton1-A rezeroed: ");
        _sb.append(rightMaster.getSelectedSensorPosition(0));
        _sb.append("u  "); // Native units
      }
    // SmtDash displays some of same info as console
    SmartDashboard.putNumber("LeftDriveDist (ft)",
        leftMaster.getSelectedSensorPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("RightDriveDist (ft)",
        rightMaster.getSelectedSensorPosition() * kDriveTick2Feet);

    // fill array to print data to console 
    _sb.append("\tVout:");
    //* x100 and cast to int to remove extra decimal value 
    _sb.append((int) (rightMaster.getMotorOutputPercent() * 100));
    _sb.append("% "); // Percent

    _sb.append("\trightMoTick: ");
    _sb.append((int)rightMaster.getSelectedSensorPosition(0));
    _sb.append("u "); // Native encoder units, 10720 = 1 wheel rot.

    _sb.append("\ttickTarg: ");
    _sb.append(targetTick);
    _sb.append("\t u ");

    if (++_loops >= 60) {
      _loops = 0;
      System.out.println(_sb.toString());
    }
    //* Empty string to get fresh data next loop 
    _sb.setLength(0);

  } // end robotPeriodic

  @Override
  public void autonomousInit() {

    enableMotorBrake(true); // sets all so neutral=Brake
    // reset encoders to zero [encod count, index0-3, timeout]
    leftMaster.setSelectedSensorPosition(0, 0, 20);
    rightMaster.setSelectedSensorPosition(0, 0, 20);
    // armMotor.setSelectedSensorPosition(0, 0, 20);

  } // end autoInit

  @Override // no defined auto Cmd, but could be called here if there was
  public void autonomousPeriodic() {
    // Auto behavior set here, drive straight 4 ft
    leftMaster.set(ControlMode.Position, targetTick);
    rightMaster.set(ControlMode.Position, targetTick);

    } // end autoPeriod

  @Override
  public void teleopInit() {
    enableMotorBrake(false); // sets all to coast
  }

  @Override
  public void teleopPeriodic() {
    // axes for game pad, left drives, right turns
    double power = -driverJoystick.getRawAxis(1); // rem: - sign
    double turn = driverJoystick.getRawAxis(4);

    // deadband set in DD class, this overrides
    if (Math.abs(power) < 0.04) {
      power = 0;
    }
    if (Math.abs(turn) < 0.04) {
      turn = 0;

    } // use stick values in DiffDrv's arcade method
    drive.arcadeDrive(power * 0.5, -turn * 0.5);

    // L-hornButton / pad's A  rezeros sensor
    // button1 = driverJoystick.getRawButton(1);
    // r-hornButton --> enable joystick(z) 2 control motor
    // button4 = driverJoystick.getRawButton(4);
    // go to preset spool position, 5
    // button5 = driverJoystick.getRawButton(5);
    // button7 = driverJoystick.getRawButton(7);

    // now should stop motion & void last cmd on re-enable
    // if (button1) { // [pos,indx,timeout]
    //   leftMaster.setSelectedSensorPosition(0, 0, 30);
    //   rightMaster.setSelectedSensorPosition(0, 0, 30);
    //   int targZeroRota = 0;
    //   leftMaster.set(ControlMode.Position, targZeroRota);
    //   rightMaster.set(ControlMode.Position, targZeroRota);
    //   _sb.append("\tButton1-A rezeroed: ");
    //   _sb.append(leftMaster.getSelectedSensorPosition(0));
    //   _sb.append("u  "); // Native units
    

    // ctre exampl: only wanted 1 press activ., used !_lastB && butt5
    // this needs constant press to drive it to target
    // if (button5) {
    // leftMaster.set(ControlMode.Position, -targetTick);
    // rightMaster.set(ControlMode.Position, -targetTick);
    // }

    // if (button7) {
    // leftMaster.set(ControlMode.Position, targetTick);
    // rightMaster.set(ControlMode.Position, targetTick);
    // }
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

    /* Save button state for on-press detection */
    // _lastButton3 = button3;
    // _lastButton4 = button4;
    // _lastButton5 = button5;

  } // end teleopPeriodic

  @Override
  public void disabledInit() {
    enableMotorBrake(false); // set true to brake & hold pos.
  }

  private void enableMotorBrake(boolean on) {
    NeutralMode mode;
    if (on) {
      mode = NeutralMode.Brake;
    } else {
      mode = NeutralMode.Coast;
    }
    // not sure if Slaves follow modes
    leftMaster.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    leftSlave.setNeutralMode(mode);
    rightSlave.setNeutralMode(mode);
    // armMotor.setNeutralMode(mode);
    // armSlave.setNeutralMode(mode);
    // rollerMotor.setNeutralMode(mode);

  } // end enableMotorBrake

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
} // end Robot.j
