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
  private ColorSensorV3 sensorLeft;
  public static boolean intakeOn = false;
  public static Joystick leftJoy = new Joystick(Constants.LEFT_JOY);
  public static Joystick rightJoy = new Joystick(Constants.RIGHT_JOY);
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
    sensorLeft = new ColorSensorV3(Port.kMXP);
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
    // State machine for auton
    switch (autonState) {
      // Go until ball finds the starting tape
      case FIND_LINE:
        int colorLeft;
        int colorRight;
        int threshold;
        if(DriverStation.getAlliance() == Alliance.Blue){
          colorRight = sensorRight.getBlue();
          colorLeft = sensorLeft.getBlue();
          threshold = Constants.BLUE_TAPE_THRESHOLD;
        } else {
          colorRight = sensorRight.getRed();
          colorLeft = sensorLeft.getRed();
          threshold = Constants.RED_TAPE_THRESHOLD;
        }

        if(colorLeft <= threshold && colorRight <= threshold) {
          autonState = AutonState.GOTO_BALL;
          drivetrain.setPosition(0);
          drivetrain.stop();
        } else {
          drivetrain.set(Constants.AUTON_SPEED);
        }
        break;
      
      // After the line, go to where we know the ball is (~40in outside of the tape)
      case GOTO_BALL:
        System.out.println(drivetrain.getPosition());
        if(drivetrain.getPosition() <= Constants.AUTON_DISTANCE ) {
          drivetrain.stop();
          autonState = AutonState.PICKUP_BALL;
        }
        else {
          drivetrain.set(Constants.AUTON_SPEED);
        }  
        break;
      
      // Turn off intake after the ball is picked up
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
