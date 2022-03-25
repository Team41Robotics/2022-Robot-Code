// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  public static boolean inUse = Intake.inUse;
  private boolean onTapeR;
  private boolean onTapeL;
  private Climber climber;
  private Shooter shooter;
  private Drivetrain drivetrain;
  private AutonState autonState;
  private ColorSensor leftColorSensor;
  private ColorSensor rightColorSensor;
  private int autonCounter;
  private NetworkTable telemetryTable;
  private long autonShootingStartTime;
  private boolean owenGlag;
  

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
    telemetryTable = NetworkTableInstance.getDefault().getTable("telemetry");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   */
  @Override
  public void robotPeriodic() {
    gatherData();
  }

  /**
   * This code runs right as auton mode is started
   */
  @Override
  public void autonomousInit() {
    autonCounter = 0;
    onTapeL = false;
    onTapeR = false;
    autonState = AutonState.ALIGN_TO_BALL;
    intake.autonInit();
    leftColorSensor.calcMedian();
    rightColorSensor.calcMedian();
    drivetrain.setPosition(0);
    if (SmartDashboard.getBoolean("Do Real Auton", true)) drivetrain.setupAlignmentToBall();
    drivetrain.stop();
    hood.home();
    Limelight.resetZoom();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (SmartDashboard.getBoolean("Do Real Auton", true)) {
      fullAuton();
    } else {
      simpleAuton();
    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    leftColorSensor.calcMedian();
    rightColorSensor.calcMedian();
    shooter.setSpeed(0);
    hood.home();
    if(inUse == false){
      inUse = true;
      intake.stop();
      inUse = false;
    }
    Limelight.setLedOn(false);
    climber.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(inUse == false){
      inUse = true;
      intake.teleop();
      inUse = false;
    }
    leftColorSensor.teleop();
    rightColorSensor.teleop();
    climber.teleop();
    shooter.teleop();
    Limelight.manualZoom(secondDS);

    double distance = Limelight.estimateDistance();
    double speed = (distance*Constants.HOOD_SPEED_SLOPE)+Constants.HOOD_SPEED_OFFSET;
    double angle = (distance*distance*Constants.HOOD_ANGLE_CURVE)+(distance*Constants.HOOD_ANGLE_SLOPE)+Constants.HOOD_ANGLE_OFFSET;


    if (secondDS.getRawButton(Controls.SecondDriverStation.INCREASE_HOOD_OFFSET)) {
      Constants.HOOD_SPEED_OFFSET += Constants.HOOD_SPEED_OFFSET_INCREMENT;
    } else if (secondDS.getRawButton(Controls.SecondDriverStation.DECREASE_HOOD_OFFSET)) {
      Constants.HOOD_SPEED_OFFSET -= Constants.HOOD_SPEED_OFFSET_INCREMENT;
    }
    if (secondDS.getRawButton(Controls.SecondDriverStation.MANUAL_SHOOTER_SPEED)) {
      shooter.setSpeed(0);
    } else if (secondDS.getRawButton(Controls.SecondDriverStation.LOW_GOAL_SETUP)) {
      Limelight.setLedOn(false);
      shooter.setSpeed(Constants.LOW_GOAL_SPEED);
      hood.setToPosition(Constants.LOW_GOAL_ANGLE);
      drivetrain.alignToGoal();
    } else if (secondDS.getRawButton(Controls.SecondDriverStation.AUTO_SHOOTING)) {
      Limelight.setLedOn(true);
      if (Limelight.targetFound()) {
        shooter.setSpeed(speed/100);
        hood.setToPosition(angle);
        drivetrain.alignToGoal();
      }
    } else if (secondDS.getRawButton(Controls.SecondDriverStation.SHOOTER_WARMUP)) {
      Limelight.setLedOn(true);
      shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
      hood.setToPosition(Constants.HOOD_DEFAULT_ANGLE);
      drivetrain.teleop();
    } else if (climber.climbing) { 
      shooter.setSpeed(0);
      Limelight.setLedOn(false);
      drivetrain.teleop();
    } else {
      drivetrain.teleop();
      hood.teleop();
      shooter.setSpeed(0);
      Limelight.setLedOn(false);
    }

    
    SmartDashboard.putNumber("LL Distance", Limelight.estimateDistance());
    SmartDashboard.putNumber("Shooter Offset", Constants.HOOD_SPEED_OFFSET);
    SmartDashboard.putBoolean("Shooter Ready", shooter.isReady());
  }
  
  /** doesnt have any code yet */
  @Override
  public void disabledInit() {}
  /**doesnt have any code yet */
  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    leftColorSensor.calcMedian();
    rightColorSensor.calcMedian();
    // hood.home();
    // drivetrain.startTime = System.currentTimeMillis();
  }

  @Override
  public void testPeriodic() {
    LiveWindow.setEnabled(false);
    System.out.println(drivetrain.getGyroAngle());
    SmartDashboard.putNumber("LL Distance", Limelight.estimateDistance());
  }

  public void fullAuton() {
    SmartDashboard.putNumber("PID Err", shooter.getErr());
    // State machine for auton
    if(inUse == false){
      inUse = true;
      switch (autonState) {
        // Go until ball finds the starting tape
        case FIND_LINE:
          shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
          if(leftColorSensor.findLineMax()){
            onTapeL = true;
            System.out.println("Left Sensor has found the tape");
            drivetrain.setLeft(0);
          }
          if(rightColorSensor.findLineMax()){
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
        
        case ALIGN_TO_BALL:
          if (drivetrain.alignToBall()) {
            drivetrain.setNoRamp(0);
            autonState = AutonState.GOTO_BALL;
          }
          break;

        case GOTO_BALL:
          // TODO: Get rid of if to always run w/ inverse kinematics and p controller
          Limelight.setLedOn(true);
          if (PhotonCamera.getArea() >= Constants.AUTON_BALL_AREA_THRESHOLD) {
            do {
              drivetrain.setPosition(0);
            }
            while (drivetrain.getPosition() > 1);
            autonState = AutonState.PICKUP_BALL;
          }
          double angle = PhotonCamera.getYaw();
          drivetrain.runInverseKinematics(AngularPController.run(-angle), Constants.AUTON_SPEED_M_PER_S);
          break;
        
        // Turn off intake after the ball is picked up
        case PICKUP_BALL:
          inUse = true;
          System.out.println(drivetrain.getPosition());
          if (drivetrain.getPosition() >= Constants.BALL_DISTANCE_FROM_BOT) {
            drivetrain.setNoRamp(0);
            // intake.run(INTAKE_MODE.OFF);
            autonState = AutonState.TRACK_GOAL;
          } else {
            drivetrain.set(Constants.AUTON_SPEED);
          }
          break;
        case TRACK_GOAL:
          if (drivetrain.alignToGoal()) {
            autonState = Constants.AutonState.PREPARE_SHOOTER;
          }
          break;

        case PREPARE_SHOOTER:
          double distance = Limelight.estimateDistance();
          double speed = (distance*Constants.HOOD_SPEED_SLOPE)+Constants.HOOD_SPEED_OFFSET+2;
          angle = (distance*distance*Constants.HOOD_ANGLE_CURVE)+(distance*Constants.HOOD_ANGLE_SLOPE)+Constants.HOOD_ANGLE_OFFSET;
          shooter.setSpeed(speed/100);
          hood.setToPosition(angle);
          if (hood.isReady() && ++autonCounter > 150) {
            autonShootingStartTime = System.currentTimeMillis();
            autonState = AutonState.SHOOT_BALL;
          }
          drivetrain.setNoRamp(0);
          break;

        case SHOOT_BALL:
          shooter.runFeeder(true);
          shooter.runElevator(Constants.ELEVATOR_FULL_SPEED);
          intake.runConveyor(true);
          if (System.currentTimeMillis() - autonShootingStartTime >= 5000) {
            shooter.runFeeder(false);
            shooter.runElevator(0);
            intake.runConveyor(false);
            autonState = AutonState.ALIGN_TO_THIRD_BALL;
          }
          break;

        case ALIGN_TO_THIRD_BALL:
          if (drivetrain.alignToBall()) {
            drivetrain.setNoRamp(0);
            intake.run(INTAKE_MODE.FORWARD);
            autonState = AutonState.GOTO_THIRD_BALL;
          }
          break;

        case GOTO_THIRD_BALL:
          if (PhotonCamera.getArea() >= Constants.AUTON_BALL_AREA_THRESHOLD) {
            System.out.println(drivetrain.getPosition());
            drivetrain.setPosition(0);
            System.out.println(drivetrain.getPosition());
            while (drivetrain.getPosition() > 1) {
              drivetrain.setPosition(0);
            }
            autonState = AutonState.PICKUP_THIRD_BALL;
          }
          else if (PhotonCamera.getYaw() >= Constants.DRIFTING_HORIZONTAL_THRESHOLD || PhotonCamera.getYaw() <= -Constants.DRIFTING_HORIZONTAL_THRESHOLD) {
            angle = PhotonCamera.getYaw();
            drivetrain.runInverseKinematics(AngularPController.run(-angle), Constants.AUTON_SPEED_M_PER_S);
          } else {
            drivetrain.set(Constants.AUTON_SPEED);
          }  
          break;
          
        case PICKUP_THIRD_BALL:
        System.out.println(drivetrain.getPosition());
          if(drivetrain.getPosition() >= Constants.BALL_DISTANCE_FROM_BOT ) {
            drivetrain.stop();
            autonState = AutonState.ALIGN_TO_GOAL_AGAIN;
          }
          else {
            drivetrain.set(Constants.AUTON_SPEED);
          }  
          break;

        case ALIGN_TO_GOAL_AGAIN:
          if (drivetrain.alignToGoal()) {
            autonState = Constants.AutonState.SHOOT_AGAIN;
          }
          break;

        case SHOOT_AGAIN:
          distance = Limelight.estimateDistance();
          speed = (distance*Constants.HOOD_SPEED_SLOPE)+Constants.HOOD_SPEED_OFFSET+2;
          angle = (distance*distance*Constants.HOOD_ANGLE_CURVE)+(distance*Constants.HOOD_ANGLE_SLOPE)+Constants.HOOD_ANGLE_OFFSET;
          shooter.setSpeed(speed/100);
          hood.setToPosition(angle);
          if (hood.isReady() && ++autonCounter > Constants.AUTON_WAIT_LOOPS) {
            autonShootingStartTime = System.currentTimeMillis();
            shooter.runFeeder(true);
            shooter.runElevator(Constants.ELEVATOR_FULL_SPEED);
            intake.runConveyor(true);
            if (System.currentTimeMillis() - autonShootingStartTime >= Constants.AUTON_SHOOTER_WAIT_TIME) {
              autonState = AutonState.ALIGN_TO_THIRD_BALL;
              shooter.runFeeder(false);
              shooter.runElevator(0);
              intake.runConveyor(false);
              Limelight.setLedOn(false);
            }
          }
          drivetrain.setNoRamp(0);
          break;

        case NONE:
          drivetrain.setNoRamp(0);
          break;
      }
      inUse = false;
    }
  }

  public void simpleAuton() {
    Limelight.setLedOn(true);
    if(drivetrain.getPosition() >= Constants.SIMPLE_AUTON_DISTANCE) {
      double distance = Limelight.estimateDistance();
      double speed = (distance*Constants.HOOD_SPEED_SLOPE)+Constants.HOOD_SPEED_OFFSET;
      double angle = (distance*distance*Constants.HOOD_ANGLE_CURVE)+(distance*Constants.HOOD_ANGLE_SLOPE)+Constants.HOOD_ANGLE_OFFSET;
      shooter.setSpeed(speed/100);
      hood.setToPosition(angle);
      if (shooter.isReady() && hood.isReady() && inUse == false) {
        inUse = true;
        shooter.runFeeder(true);
        intake.runConveyor(true);
        inUse = false;
      }
    }
    else {
      drivetrain.set(Constants.AUTON_SPEED);
    } 
  }

  public void gatherData() {
    owenGlag = telemetryTable.getEntry("owenGlag").getBoolean(true);
    if(owenGlag){
      telemetryTable.getEntry("time").setDouble(System.currentTimeMillis());
      climber.telemetry(telemetryTable);
      leftColorSensor.telemetry(telemetryTable, "Left Color Sensor");
      rightColorSensor.telemetry(telemetryTable, "Right Color Sensor");
      drivetrain.telemetry(telemetryTable);
      hood.telemetry(telemetryTable);
      intake.telemetry(telemetryTable);
      shooter.telemetry(telemetryTable);
      telemetryTable.getEntry("owenGlag").setBoolean(false);
  }
  }
}
