// armProtoFlatA                           Robot.j
// button control always failed,, joystk OK
// import of flat template Romi base for arm prototype -- more
// code from closeLoopPos-REV --> live k_ update from SmtDash
// which also has simple Cmd/Subs features, not used here

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The VM is configured to automatically run this class, calling the
 * functions corresponding to each mode, as described in TimedRobot docs.
 * all robot config set here in Robot.j -- (normally few specifics here
 * if using RoboCont and Subsyst.)
 */
public class Robot extends TimedRobot {

  // instance joystick @ 0 --assumes xBox pad plugged into USB 0
  private final XboxController myJoystick = new XboxController(0);

  // single brushed motor, PWM control (port) of Spark by RoboRIO
  private final Spark armMotor = new Spark(0);

  // if reading Quad encoder on arm, hardwire A,B to RIO DIO port 0,1
  // public static Encoder armEncoder = new Encoder(0,1);

  // for REV thru bore 11-1271 using abs. mode full 360° rot = 1024 count
  // remote absol. encod on arm axis, wired to RIO port
  DutyCycleEncoder absolArmEncod = new DutyCycleEncoder(0);

  // angle control setpoints, used by button triggers
  public static double setpointA = 6;
  public static double setpointB = 60;
  public static double setpointX = 120;
  public static double setpointY = 170; // end limit =~180°

  // angle controller PID param def as class var
  public static double kP = 0.05;
  public static double kI = 0.000;
  public static double kD = 0.00;
  // not used yet
  // public static double kMaxOutput = 0.1;
  // public static double kMinOutput = 0.1;

  // Trigger leftBump; // class var, used in >1 method
  // Trigger buttonA;
  // Trigger buttonB;
  // Trigger buttonX;
  // Trigger buttonY;

  // PID system to rotate arm to discrete angle with button;
  private final PIDController angleControl = new PIDController(kP, kI, kD);

  /**
   * roboInit runs (once) when the robot is first started and sets most
   * robot specifics (things done in RoboCont if cmd/subsys framwk)
   */
  @Override
  public void robotInit() {

    // invert / or not, so that positive V (to red motor input) results in
    // arm moving "forward'. Normal polarity goes fwd on prototype but
    // absolute encoder reads 'fwd' as negative, for RIO control I need
    // to invert. I need to reset() manually after manually going full fwd
    // which is 'reverse' to RIO code & gamepad
    armMotor.setInverted(true);

    // set 'distance' (angle°) convers. factor for absol. encod.
    absolArmEncod.setDistancePerRotation(360.0);

    // to read out in rad.
    // absolArmEncod.setDistancePerRotation(2 * Math.PI);

    // the delta tolerance ensures arm is stable at the
    // setpoint before it's counted as at the reference (setpt)
    angleControl.setTolerance(2);
    // deg, deg/sec (dist , vel)

    // Resets encoder to output distance of 0 at the current position
    // leftBump = new Trigger(myJoystick::getLeftBumperPressed); // may prefer
    // Pressed

    // leftBump.onTrue(new PrintCommand("armEnco reset"));
    // reset encod trigger failed here; joystk method works in rP, all modes
    // leftBump.onTrue(new PrintCommand("armEnco reset"))
    // .onTrue(new InstantCommand(() -> absolArmEncod.reset()));

    // buttonA = new Trigger(myJoystick::getAButton);

    // buttonB = new Trigger(myJoystick::getBButton);

    // buttonX = new Trigger(myJoystick::getXButton);

    // buttonY = new Trigger(myJoystick::getYButton);

    // move angleGoal on SmtDash using var-->rP
    // buttonY
    // // .onTrue(new RunCommand(() ->
    // // armMotor.set(angleControl.calculate(absolArmEncod.getDistance(),
    // // setpointY))))
    // .onTrue(new PrintCommand("set to angle Y"))
    // .onTrue(new InstantCommand(() -> SmartDashboard.putNumber("angleGoal",
    // setpointY)));

    // Trigger armRevrs = new JoystickButton(myJoystick, povleft);
    // armRevrs.whileTrue(armMotor.set(- 0.2));

    Trigger armRevrs = new POVButton(myJoystick, 270);
    Trigger armForwd = new POVButton(myJoystick, 90);

    // work here but only when bot Enabled, hold button to keep moving
    armForwd.onTrue(new InstantCommand(() -> armMotor.set(0.35)));
    armForwd.onFalse(new InstantCommand(() -> armMotor.set(0.0)));
    // these onTrue started movement, didn't stop until .onFalse added
    armRevrs.onTrue(new InstantCommand(() -> armMotor.set(-0.25))); // slower
    armRevrs.onFalse(new InstantCommand(() -> armMotor.set(0.0)));

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);

    // init field for desired position setpoint and present angle
    SmartDashboard.putNumber("angleGoal", 0);
    // to show present encoder value
    SmartDashboard.putNumber("armAngle deg.", 0);
    // to put some text
    // SmartDashboard.putString("GTPcmd fin?", "???");
  } // end robotInit()

  // This function is called every robot packet, no matter the mode.
  @Override
  public void robotPeriodic() { // normally, in cmd/subsys paradigm:
    // calls the Scheduler <-- responsible for polling buttons, adding
    // newly-scheduled commands, running now-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodics.
    // Essential for Command-based framework to work.
    // ... in flat framework will run essential stuff regardless of mode
    CommandScheduler.getInstance().run();

    // trigger didn't work in rP or rI; this works all mode
    // leftBump.onTrue(new PrintCommand("armEnco reset"))
    // /* keep line indent */ .onTrue(new InstantCommand(() ->
    // absolArmEncod.reset()));
    // this works all mode
    if (myJoystick.getRawButton(5))
      absolArmEncod.reset();

    // buttonA
    // .whileTrue(
    // new InstantCommand(() -> armMotor.set(angleControl.calculate
    // /* keep line indent */ (absolArmEncod.getDistance(), setpointA) * 0.1)))
    // .onTrue(new PrintCommand("arm to A angle"));

    // buttonB in tp ran, didn't stop
    if (myJoystick.getRawButton(2)) {
      armMotor.set(angleControl.calculate // 60 deg
      /* keep line indent */ (absolArmEncod.getDistance(), setpointB) * 0.05);
      new PrintCommand("set to angle B");
    }
    // same
    if (myJoystick.getRawButton(3)) {
      armMotor.set(angleControl.calculate
      /* keep line indent */ (absolArmEncod.getDistance(), setpointX) * 0.05);
      new PrintCommand("set to angle X");
    }

    if (myJoystick.getRawButton(4)) {
      armMotor.set(angleControl.calculate
      /* keep line indent */ (absolArmEncod.getDistance(), setpointY) * 0.05);
      new PrintCommand("set to angle Y");
    }

    if (angleControl.atSetpoint()) {
      armMotor.set(0.0);
      angleControl.close();
    }

    SmartDashboard.putNumber("armAngle deg.", Math.round(absolArmEncod.getDistance()));

  } // end robotPeriodic

  // autoInit gets the autonomous command name set by SmartDashbd
  @Override
  public void autonomousInit() {
  } // end autoInit

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  } // end autoPeriod

  @Override
  public void teleopInit() { // ? if triggers will run w/o code here or tP

    // This confirms that the autonomous code has stopped. If you want
    // auto cmd to continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autoSelected != null) { // if auto never active, don't do resets
    // m_autoSelected.cancel();
    // end if

  } // end teleInit

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // execute button action et stick here, prn (if rP doesn't)
    // trigger button instanced in rI, actions here

  } // end telePeri

  // This function is called once each time the robot enters Disabled mode.
  @Override
  public void disabledInit() {
    // CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
} // end class
