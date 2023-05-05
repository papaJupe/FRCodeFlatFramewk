// chewbaccaArcadeA : arcade drive w/ 4 Neo on CA drive, flat framework -- 
(all in Robot.j), drives only, no Auto, edited AM 221204 from REV example Motor_Follow

// Copyright (c) FIRST and other WPILib contributors.
// Open Source; you can modify and/or share it under the terms of
// the WPILib BSD license file in root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * ...shows the use of the DifferentialDrive class. Runs the
 * motors with split arcade steering and Xbox controller.
 */
public class Robot extends TimedRobot {
  // declare class vars for hardware, base drive class

  // CAN bus ID of motor
  private static final int leftMasterID = 3;
  private static final int leftSlaveID = 4;
  private static final int rightMasterID = 1;
  private static final int rightSlaveID = 2;

  // 4 Neo motor
  private CANSparkMax my_leftMaster;
  private CANSparkMax my_leftSlave;
  private CANSparkMax my_rightMaster;
  private CANSparkMax my_rightSlave;

  private DifferentialDrive my_DiffDrive;

  private XboxController my_driveJoy;

  @Override
  public void robotInit() {

    /*
     * instance motor, joystick, drive objects
     * This initializes 2 brushless Masters (Neos) with CAN IDs
     * and 2 Slave followers (Neos). Change IDs for your setup.
     */
    my_leftMaster = new CANSparkMax(leftMasterID, MotorType.kBrushless);
    my_leftSlave = new CANSparkMax(leftSlaveID, MotorType.kBrushless);
    my_rightMaster = new CANSparkMax(rightMasterID, MotorType.kBrushless);
    my_rightSlave = new CANSparkMax(rightSlaveID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration
     * parameters of the SPARK MAX to their factory default state. If no argument
     * is passed, these parameters will not persist between power cycles
     */
    my_leftMaster.restoreFactoryDefaults();
    my_leftSlave.restoreFactoryDefaults();
    my_rightMaster.restoreFactoryDefaults();
    my_rightSlave.restoreFactoryDefaults();

    // invert R side?, example says:
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side.
    //

    my_leftMaster.setInverted(false);
    my_rightMaster.setInverted(true);

    // slaves follow, probably follow setInversion too
    my_leftSlave.follow(my_leftMaster);
    my_rightSlave.follow(my_rightMaster);

    my_leftMaster.setOpenLoopRampRate(0.5);
    my_rightMaster.setOpenLoopRampRate(0.5);

    my_DiffDrive = new DifferentialDrive(my_leftMaster, my_rightMaster);

    my_driveJoy = new XboxController(0);

  } // end robotInit

  @Override
  public void teleopPeriodic() {
    // Drive with split arcade drive.
    // i.e. here the Y axis of the left stick powers forward
    // and backward, and the X of the right stick turns left and right.
    // drive class has default deadband 0.02, change with .setDeadband(dbl)
    my_DiffDrive.arcadeDrive(-my_driveJoy.getLeftY() * 0.6, my_driveJoy.getRightX() * 0.3);

  } // end teleopPeriodic
}  // end robot.j
