// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;


import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Constants.AutonState;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private ColorSensorV3 sensorRight;
  public static boolean onTape = false;
  public static boolean intakeOn = false;
  public static Joystick leftJoy = new Joystick(1);
  public static Joystick rightJoy = new Joystick(0);
  private Drivetrain drivetrain;
  private AutonState autonState;
  private Intake intake;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    sensorRight = new ColorSensorV3(Port.kOnboard);
    drivetrain = new Drivetrain();
    intake = new Intake();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   */
  @Override
  public void autonomousInit() {
    autonState = AutonState.FIND_LINE;
    intake.autonInit();
  }

  /* This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (autonState) {
      case FIND_LINE:
        int colorValue;
        int threshold;
        if(DriverStation.getAlliance() == Alliance.Blue){
          colorValue = sensorRight.getBlue();
          threshold = 950;
        } else {
          colorValue = sensorRight.getRed();
          threshold = 1000;
        }

        if(colorValue <= threshold){
          autonState = AutonState.GOTO_BALL;
          drivetrain.setPosition(0);
          drivetrain.stop();
        } else {
          drivetrain.set(-0.1);
        }
        break;
      
      case GOTO_BALL:
        System.out.println(drivetrain.getPosition());
        if(drivetrain.getPosition() <=-10 ) {
          drivetrain.stop();
          autonState = AutonState.PICKUP_BALL;
        }
        else {
          drivetrain.set(-.1);
        }  
        break;
      
      case PICKUP_BALL:
        intake.setIntakeMotor(0);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drivetrain.teleop();
    intake.teleop();
    System.out.println(Integer.toString(sensorRight.getRed()) + "," + Integer.toString(sensorRight.getGreen()) + "," + Integer.toString(sensorRight.getBlue()));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
