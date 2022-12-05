
/* Copyright (c) 2018 FIRST. All Rights Reserved.              */
/* Open Source Software - may be modified and shared by FRC teams.*/
/* must have the FIRST BSD license file in the root directory */
                                                              
/* imported from SunCode#2timedRobot, simple flat framework, edited 221123 for 
Flatbot, single joystick, TalonSRX x4 CA drive, auto from encoder distance, 
no PID, motor encoder values to smart dashboard, all params may need retune */

package frc.robot;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. [If you change the name of this class or the package] <--
 * just don't.
 */
public class Robot extends TimedRobot {
  /**
   * inits run when the robot is started
   */

  // actuators
  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(4);
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(2);
  private WPI_TalonSRX leftSlave = new WPI_TalonSRX(3);
  private WPI_TalonSRX rightSlave = new WPI_TalonSRX(1);

  // private WPI_TalonSRX/ armMotor = new WPI_TalonSRX(5);
  // private WPI_TalonSRX armSlave = new WPI_TalonSRX(3);

  // private WPI_TalonSRX rollerMotor = new WPI_TalonSRX(4);

  // // private Compressor compressor = new Compressor(null);
  // private DoubleSolenoid hatchIntake = new
  // DoubleSolenoid(PneumaticsModuleType.CTREPCM,
  // 0, 1); // PCM port 0, 1

  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  // joysticks
  private Joystick driverJoystick = new Joystick(0);
  // private Joystick operatorJoystick = new Joystick(1);

  // unit conversion
  private final double kDriveTick2Feet = 1.0 / 10700 * 6 * Math.PI / 12;
  // private final double kArmTick2Deg = 360.0 / 512 * 26 / 42 * 18 /
  // 60 * 18 / 84;

  @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber("Arm Encoder Value",//
    // armMotor.getSelectedSensorPosition() * kArmTick2Deg);

    SmartDashboard.putNumber("Left Drive Encoder",
        leftMaster.getSelectedSensorPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("Right Drive Encoder",
        rightMaster.getSelectedSensorPosition() * kDriveTick2Feet);
  }

  @Override
  public void robotInit() {
    // inverted settings suspect one or other should be false
    leftMaster.setInverted(false);
    rightMaster.setInverted(true);
    // armMotor.setInverted(false);

    // slave setups
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    // armSlave.follow//(armMotor);
    // should follow, so maybe not needed?
    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
    // armSlave.setInverted(InvertType.FollowMaster);

    // init encoders
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
     0, 20);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
     0, 20);
    // armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    /*
     * Sets the phase of the sensor. Use when controller forward/reverse output
     * doesn't correlate to appropriate forward/reverse reading of sensor.
     * Pick a value so that positive PercentOutput yields a positive change in
     * sensor.
     * After setting this, user can freely call SetInverted() with any value.
     */
    leftMaster.setSensorPhase(false);
    rightMaster.setSensorPhase(true);
    // armMotor.setSensorPhase(true);

    // reset encoders to zero
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
    // armMotor.setSelectedSensorPosition(0, 0, 10);

    // set encoder boundary limits: to stop motors
    // armMotor.configReverseSoftLimitThreshold((int) (0 / kArmTick2Deg), 10);
    // armMotor.configForwardSoftLimitThreshold((int) (175 / kArmTick2Deg), 10);
    // armMotor.configReverseSoftLimitEnable(true, 10);
    // armMotor.configForwardSoftLimitEnable(true, 10);
    // start compressor, automatic now
    // compressor.start();

    leftMaster.configOpenloopRamp(1.0, 10);
    rightMaster.configOpenloopRamp(1.0, 10);

    drive.setDeadband(0.05);

  }   // end robotInit

  @Override
  public void autonomousInit() {
    enableMotors(true); // sets to Brake
    // reset encoders to zero
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
    // armMotor.setSelectedSensorPosition(0, 0, 10);

  }

  @Override
  public void autonomousPeriodic() {
    double leftPosition = leftMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    double rightPosition = rightMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    double distance = (leftPosition + rightPosition) / 2;

    if (distance < 4) {
      drive.tankDrive(0.6, 0.6);
    } else {
      drive.tankDrive(0, 0);
    }
  }

  @Override
  public void teleopInit() {
    enableMotors(true); // sets to Brake
  }

  @Override
  public void teleopPeriodic() {
    // driving
    double power = -driverJoystick.getRawAxis(1); // remember: negative sign
    double turn = driverJoystick.getRawAxis(4);
    // deadband
    if (Math.abs(power) < 0.05) {
      power = 0;
    }
    if (Math.abs(turn) < 0.05) {
      turn = 0;
    }
    drive.arcadeDrive(power * 0.6, turn * 0.3);

    // arm control
    // double armPower = -operatorJoystick.getRawAxis(1); // remember negative sign
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
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {
    enableMotors(false);
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
  }
}
