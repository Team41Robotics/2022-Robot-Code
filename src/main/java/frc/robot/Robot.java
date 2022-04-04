// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
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
  public static long startTime;
  public static boolean inUse = Intake.inUse;
  private UsbCamera cam;
  private Climber climber;
  private Shooter shooter;
  private DigitalInput beamBreak;
  private Drivetrain drivetrain;
  private AutonState autonState;
  private NetworkTable telemetryTable;
  private long autonShootingStartTime, autonHumanStationWait;
  private boolean owenGlag, thirdBallClose, started;
  private int dropCount;
  private double robotPos;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    started = false;
    drivetrain = new Drivetrain();
    intake = new Intake();
    shooter = new Shooter();
    hood = new Hood();
    climber = new Climber();
    telemetryTable = NetworkTableInstance.getDefault().getTable("telemetry");
    cam = CameraServer.startAutomaticCapture();
    cam.setExposureAuto();
    cam.setVideoMode(VideoMode.PixelFormat.kMJPEG, 80, 60, 30);
    cam.setFPS(30);
    startTime = System.currentTimeMillis();
    beamBreak = new DigitalInput(4);
    autonState = AutonState.NONE;
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
    autonState = AutonState.ALIGN_TO_BALL;
    started = true;
    thirdBallClose = false;
    intake.autonInit();
    drivetrain.setPosition(0);
    if (SmartDashboard.getBoolean("Do Real Auton", true)) drivetrain.setupAlignmentToBall();
    drivetrain.stop();
    hood.home();
    shooter.setSpeed(0);
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
    telemetryTable.getEntry("disabled").setBoolean(false);
    autonState = AutonState.NONE;
    shooter.setSpeed(0);
    hood.home();
    intake.reset();
    Limelight.setLedOn(false);
    Limelight.resetZoom();
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
    climber.teleop();
    shooter.teleop();
    Limelight.manualZoom(secondDS);

    double distance = Limelight.estimateDistance();
    double speed = (distance * Constants.HOOD_SPEED_SLOPE) + Constants.HOOD_SPEED_OFFSET;
    double angle = (distance * distance * Constants.HOOD_ANGLE_CURVE) + (distance * Constants.HOOD_ANGLE_SLOPE)
        + Constants.HOOD_ANGLE_OFFSET;

    if (secondDS.getRawButtonPressed(Controls.SecondDriverStation.SHOOTER_OFFSET_UP)) {
      Constants.HOOD_SPEED_OFFSET += 1;
    } else if (secondDS.getRawButtonPressed(Controls.SecondDriverStation.SHOOTER_OFFSET_DOWN)) {
      Constants.HOOD_SPEED_OFFSET -= 1;
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
    SmartDashboard.putBoolean("Beam Break", !beamBreak.get());
  }
  
  /** doesnt have any code yet */
  @Override
  public void disabledInit() {}
  /**doesnt have any code yet */
  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    // hood.home();
    // drivetrain.startTime = System.currentTimeMillis();
  }

  @Override
  public void testPeriodic() {
    LiveWindow.setEnabled(false);
    System.out.println(drivetrain.getGyroAngle());
    SmartDashboard.putBoolean("Beam Break", !beamBreak.get());
  }

  public void fullAuton() {
    SmartDashboard.putNumber("PID Err", shooter.getErr());
    // State machine for auton
    if(inUse == false){
      inUse = true;
      switch (autonState) {
        // Go until ball finds the starting tape
        case FIND_LINE:
          break;
        
        // After the line, go to where we know the ball is (~40in outside of the tape)
        
        case ALIGN_TO_BALL:
          shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
          hood.home();
          if (drivetrain.alignToBall()) {
            drivetrain.setNoRamp(0);
            autonState = AutonState.GOTO_BALL;
          }
          break;

        case GOTO_BALL:
          shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
          hood.home();
          Limelight.setLedOn(true);
          // Beam break here
          if (!beamBreak.get()) {
            do {
              drivetrain.setPosition(0);
            }
            while (drivetrain.getPosition() > 1);
            autonState = AutonState.TRACK_GOAL;
          }
          double angle = PhotonCamera.getYaw();
          drivetrain.runInverseKinematics(AngularPController.run(-angle), Constants.AUTON_SPEED_M_PER_S*(2.0/3));
          break;
        
        // Turn off intake after the ball is picked up
        case PICKUP_BALL:
          shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
          hood.home();
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
          shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
          hood.home();
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
          if (hood.isReady()) {
            autonShootingStartTime = System.currentTimeMillis();
            autonState = AutonState.SHOOT_BALL;
          }
          drivetrain.setNoRamp(0);
          break;

        case SHOOT_BALL:
          shooter.runFeeder(true);
          shooter.runElevator(Constants.ELEVATOR_FULL_SPEED);
          intake.runConveyor(true);
          if (System.currentTimeMillis() - autonShootingStartTime >= Constants.AUTON_SHOOTER_WAIT_TIME) {
            shooter.runFeeder(false);
            shooter.runElevator(0);
            intake.runConveyor(false);
            autonState = AutonState.ALIGN_TO_THIRD_BALL;
          }
          break;

        case ALIGN_TO_THIRD_BALL:
          Limelight.zoomIn();
          if (drivetrain.alignToBall()) {
            drivetrain.setNoRamp(0);
            intake.run(INTAKE_MODE.FORWARD);
            autonState = AutonState.GOTO_THIRD_BALL;
          }
          break;

        case GOTO_THIRD_BALL:
          if (drivetrain.getDanger()){
            autonState = AutonState.WAIT_FOR_BALL;
          }
          if (!beamBreak.get()) {
            drivetrain.setNoRamp(0);
            // System.out.println(drivetrain.getPosition());
            // drivetrain.setPosition(0);
            // System.out.println(drivetrain.getPosition());
            // drivetrain.setPosition(0);
            autonHumanStationWait = System.currentTimeMillis();
            robotPos = drivetrain.getPosition();
            autonState = AutonState.WAIT_FOR_BALL;
          } else {
            if (PhotonCamera.getArea() > 5) {
              thirdBallClose = true;
            }
            angle = PhotonCamera.getYaw();
            double slowSpeed = 0.5;
            drivetrain.runInverseKinematics(AngularPController.run(-angle), !thirdBallClose ? Constants.AUTON_SPEED_M_PER_S : slowSpeed);
          }
          break;
          
        case WAIT_FOR_BALL:
          if (System.currentTimeMillis()-autonHumanStationWait > Constants.AUTON_HUMAN_BALL_WAIT_TIME) {
            autonState = AutonState.MOVE_TOWARDS_GOAL;
          } else {
            drivetrain.setNoRamp(0);
          }
          break;
        
        case MOVE_TOWARDS_GOAL:
          if(Math.abs(drivetrain.getPosition()-robotPos) >= Constants.DISTANCE_FROM_HUMAN_STATION ) {
            do drivetrain.stop(); while (!drivetrain.isReady());
            autonState = AutonState.ALIGN_TO_GOAL_AGAIN;
          }
          else {
            drivetrain.runInverseKinematics(0, -Constants.AUTON_SPEED_M_PER_S);
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
          if (hood.isReady()) {
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
      telemetryTable.getEntry("time").setDouble(System.currentTimeMillis()-startTime);
      climber.telemetry(telemetryTable);
      drivetrain.telemetry(telemetryTable);
      hood.telemetry(telemetryTable);
      intake.telemetry(telemetryTable);
      shooter.telemetry(telemetryTable);
      telemetryTable.getEntry("autonState").setNumber(autonStateToInt());
      telemetryTable.getEntry("started").setBoolean(started);
      telemetryTable.getEntry("disabled").setBoolean(RobotState.isDisabled());
      telemetryTable.getEntry("climbingState").setDouble(climber.getClimbingState());
      telemetryTable.getSubTable("cameras").getSubTable("photonvision").getEntry("angle").setDouble(PhotonCamera.getYaw());
      telemetryTable.getSubTable("cameras").getSubTable("photonvision").getEntry("area").setDouble(PhotonCamera.getArea());
      telemetryTable.getSubTable("cameras").getSubTable("limelight").getEntry("angle").setDouble(Limelight.getHorizontalAngle());
      telemetryTable.getEntry("owenGlag").setBoolean(false);
      dropCount = 0;;
    }else{
      dropCount += 1;
      // System.out.println(String.format("Message dropped... Times Since Last Success: %s",dropCount));
    }
  }

  public int autonStateToInt() {
    switch (autonState) {
      case NONE:
        return 0;
      case FIND_LINE:
        return 1;
      case ALIGN_TO_BALL:
        return 2;
      case GOTO_BALL:
        return 3;
      case PICKUP_BALL:
        return 4;
      case TRACK_GOAL:
        return 5;
      case PREPARE_SHOOTER:
        return 6;
      case SHOOT_BALL:
        return 7;
      case ALIGN_TO_THIRD_BALL:
        return 8;
      case GOTO_THIRD_BALL:
        return 9;
      case MOVE_TOWARDS_GOAL:
        return 10;
      case ALIGN_TO_GOAL_AGAIN:
        return 11;
      case SHOOT_AGAIN:
        return 12;
      case WAIT_FOR_BALL:
        return 13;
      default:
        return -1;
    }
  }
}
