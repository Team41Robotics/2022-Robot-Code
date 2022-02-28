// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.ColorSensorV3;
import frc.robot.Constants.AutonState;
import frc.robot.Constants.INTAKE_MODE;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static boolean intakeOn = false;
  public static Joystick leftJoy = new Joystick(Constants.LEFT_JOY);
  public static Joystick rightJoy = new Joystick(Constants.RIGHT_JOY);
  public static Joystick secondDS = new Joystick(Constants.RIGHT_DRIVER_STATION);
  public static Intake intake;
  public static Hood hood;
  private boolean onTapeR;
  private boolean onTapeL;
  private Climber climber;
  private Shooter shooter;
  private Drivetrain drivetrain;
  private AutonState autonState;
  private ColorSensor leftColorSensor;
  private ColorSensor rightColorSensor;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    drivetrain = new Drivetrain();
    intake = new Intake();
    leftColorSensor = new ColorSensor(new ColorSensorV3(Port.kMXP));
    rightColorSensor = new ColorSensor(new ColorSensorV3(Port.kOnboard));
    shooter = new Shooter();
    hood = new Hood();
    climber = new Climber();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This code runs right as auton mode is started
   */
  @Override
  public void autonomousInit() {
    onTapeL = false;
    onTapeR = false;
    autonState = AutonState.FIND_LINE;
    intake.autonInit();
    leftColorSensor.calcMedian();
    rightColorSensor.calcMedian();
    drivetrain.setPosition(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // State machine for auton
    autonState = AutonState.NONE;
    switch (autonState) {
      // Go until ball finds the starting tape
      case FIND_LINE:
        if(leftColorSensor.findLine()){
          onTapeL = true;
          System.out.println("Left Sensor has found the tape");
          drivetrain.setLeft(0);
        }
        if(rightColorSensor.findLine()){
          onTapeR = true;
         System.out.println("Right Sensor has found tape");
          drivetrain.setRight(0);
        }    
        if(!onTapeL){
          drivetrain.setLeft(Constants.AUTON_SPEED);
        }
        if(!onTapeR){
          drivetrain.setRight(Constants.AUTON_SPEED);
        }
        if(onTapeL && onTapeR) {
          autonState = AutonState.GOTO_BALL;
          drivetrain.setPosition(0);
          drivetrain.stop();
        }
        break;
      
      // After the line, go to where we know the ball is (~40in outside of the tape)
      case GOTO_BALL:
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
        intake.run(INTAKE_MODE.FORWARD);
        autonState = AutonState.TRACK_BALL;
        break;
        
      case TRACK_BALL:
        Limelight.setLedOn(true);
        drivetrain.alignToGoal();
        break;

      case NONE:
        drivetrain.runInverseKinematics(.25, -0.5);
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    leftColorSensor.calcMedian();
    rightColorSensor.calcMedian();
    shooter.setSpeed(0);
    hood.home();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drivetrain.teleop();
    intake.teleop();
    leftColorSensor.teleop();
    rightColorSensor.teleop();
    climber.teleop();
    shooter.teleop();

    double distance = Limelight.estimateDistance();
    double speed = (distance*Constants.HOOD_MID_SPEED_SLOPE)+Constants.HOOD_MID_SPEED_OFFSET;
    double angle = (distance*Constants.HOOD_MID_ANGLE_SLOPE)+Constants.HOOD_MID_ANGLE_OFFSET;

    if (secondDS.getRawButton(6)) {
      shooter.setSpeed(speed/100);
      hood.setToPosition(angle);
      System.out.print("Speed: ");
      System.out.print(speed/100);
      System.out.print("\t\tAngle: ");
      System.out.println(angle);
    } else {
      hood.teleop();
      shooter.setSpeed(0);
    }
  }
  
  /** doesnt have any code yet */
  @Override
  public void disabledInit() {}
  /**doesnt have any code yet */
  @Override
  public void disabledPeriodic() {}
  /**doesnt have any code yet */
  @Override
  public void testInit() {
    hood.home();
  }
  /**doesnt have any code yet */
  @Override
  public void testPeriodic() {
    // intake.test();
    hood.test();
    shooter.test();
    // Limelight.test();
    System.out.print("Hood Angle: ");
    System.out.print(hood.angle);
    System.out.print("\t\tShooter Speed: ");
    System.out.println(shooter.speed);
  }
}
