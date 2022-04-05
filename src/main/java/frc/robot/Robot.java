// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.AutonState;
import frc.robot.Constants.INTAKE_MODE;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static boolean intakeOn = false;
  public static Joystick leftJoy = new Joystick(Constants.LEFT_JOY);
  public static Joystick rightJoy = new Joystick(Constants.RIGHT_JOY);
  public static Joystick secondDS = new Joystick(Constants.RIGHT_DRIVER_STATION);
  public static Hood hood;
  public static long startTime;
  public static boolean inUse = Intake.inUse;
  private Climber climber;
  private Shooter shooter;
  private DigitalInput beamBreak;
  private AutonState autonState;
  private NetworkTable telemetryTable;
  private NetworkTableEntry limelightDistanceEntry, speedOffsetEntry, shooterReadyEntry, beamBreakEntry, droppedTelemetryEntry;
  private SendableChooser<Boolean> autonChooser;
  private ShuffleboardTab debugTab, robotTab;
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
    shooter = new Shooter();
    Hood.initHood();
    climber = new Climber();
    Drivetrain.initDrivetrain();
    Intake.initialize();
    telemetryTable = NetworkTableInstance.getDefault().getTable("telemetry");
    startTime = System.currentTimeMillis();
    beamBreak = new DigitalInput(4);
    autonState = AutonState.NONE;
    debugTab = Shuffleboard.getTab("Debug");
    robotTab = Shuffleboard.getTab("Robot");
    limelightDistanceEntry = debugTab.add("LL Distance", -1).getEntry();
    speedOffsetEntry = robotTab.add("Speed Offset", Constants.HOOD_SPEED_OFFSET).getEntry();
    shooterReadyEntry = robotTab.add("Shooter Ready", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    beamBreakEntry = debugTab.add("Beam Break", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    droppedTelemetryEntry = debugTab.add("Telemetry Packets Dropped", -1).getEntry();

    autonChooser = new SendableChooser<>();
    autonChooser.setDefaultOption("4 Ball Auton", true);
    autonChooser.addOption("2 Ball Auton", false);
    robotTab.add("Do Real Auton", autonChooser).withWidget(BuiltInWidgets.kComboBoxChooser); // true: 4 ball   false: 2 ball
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
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
    PhotonCamera.setPipeline(DriverStation.getAlliance() == Alliance.Blue);
    telemetryTable.getEntry("disabled").setBoolean(false);
    autonState = AutonState.ALIGN_TO_BALL;
    started = true;
    thirdBallClose = false;
    Intake.autonInit();
    Drivetrain.setPosition(0);
    Drivetrain.setupAlignmentToBall();
    Drivetrain.stop();
    Hood.home();
    shooter.setSpeed(0);
    Limelight.resetZoom();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (autonChooser.getSelected()) {
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
    Hood.home();
    Intake.reset();
    Limelight.setLedOn(false);
    Limelight.resetZoom();
    climber.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (inUse == false) {
      inUse = true;
      Intake.teleop();
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
      Hood.setToPosition(Constants.LOW_GOAL_ANGLE);
      Drivetrain.alignToGoal();
    } else if (secondDS.getRawButton(Controls.SecondDriverStation.AUTO_SHOOTING)) {
      Limelight.setLedOn(true);
      if (Limelight.targetFound()) {
        shooter.setSpeed(speed / 100);
        Hood.setToPosition(angle);
        Drivetrain.alignToGoal();
      }
    } else if (secondDS.getRawButton(Controls.SecondDriverStation.SHOOTER_WARMUP)) {
      Limelight.setLedOn(true);
      shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
      Hood.setToPosition(Constants.HOOD_DEFAULT_ANGLE);
      Drivetrain.teleop();
    } else if (climber.climbing) {
      shooter.setSpeed(0);
      Limelight.setLedOn(false);
      Drivetrain.teleop();
    } else {
      Drivetrain.teleop();
      Hood.teleop();
      shooter.setSpeed(0);
      Limelight.setLedOn(false);
    }
    limelightDistanceEntry.setDouble(Limelight.estimateDistance());
    speedOffsetEntry.setDouble(Constants.HOOD_SPEED_OFFSET);
    shooterReadyEntry.setBoolean(shooter.isReady());
    beamBreakEntry.setBoolean(!beamBreak.get());
  }

  /** doesnt have any code yet */
  @Override
  public void disabledInit() {
  }

  /** doesnt have any code yet */
  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    telemetryTable.getEntry("disabled").setBoolean(false);
    // hood.home();
    // drivetrain.startTime = System.currentTimeMillis();
  }

  @Override
  public void testPeriodic() {
    LiveWindow.setEnabled(false);
    System.out.println(autonChooser.getSelected());
  }

  public void fullAuton() {
    // State machine for auton
    if (inUse == false) {
      inUse = true;
      switch (autonState) {
        // Go until ball finds the starting tape
        case FIND_LINE:
          break;

        // After the line, go to where we know the ball is (~40in outside of the tape)

        case ALIGN_TO_BALL:
          shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
          Hood.home();
          if (Drivetrain.alignToBall()) {
            Drivetrain.setNoRamp(0);
            autonState = AutonState.GOTO_BALL;
          }
          break;

        case GOTO_BALL:
          shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
          Hood.home();
          Limelight.setLedOn(true);
          // Beam break here
          if (!beamBreak.get()) {
            do {
              Drivetrain.setPosition(0);
            } while (Drivetrain.getPosition() > 1);
            autonState = AutonState.TRACK_GOAL;
          }
          double angle = PhotonCamera.getYaw();
          Drivetrain.runInverseKinematics(Constants.BALL_FOLLOWING_kP * -angle, Constants.AUTON_SPEED_M_PER_S * (2.0 / 3));
          break;

        // Turn off intake after the ball is picked up
        case PICKUP_BALL:
          shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
          Hood.home();
          inUse = true;
          System.out.println(Drivetrain.getPosition());
          if (Drivetrain.getPosition() >= Constants.BALL_DISTANCE_FROM_BOT) {
            Drivetrain.setNoRamp(0);
            // intake.run(INTAKE_MODE.OFF);
            autonState = AutonState.TRACK_GOAL;
          } else {
            Drivetrain.set(Constants.AUTON_SPEED);
          }
          break;
        case TRACK_GOAL:
          shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
          Hood.home();
          if (Drivetrain.alignToGoal()) {
            autonState = Constants.AutonState.PREPARE_SHOOTER;
          }
          break;

        case PREPARE_SHOOTER:
          double distance = Limelight.estimateDistance();
          double speed = (distance * Constants.HOOD_SPEED_SLOPE) + Constants.HOOD_SPEED_OFFSET + .5;
          angle = (distance * distance * Constants.HOOD_ANGLE_CURVE) + (distance * Constants.HOOD_ANGLE_SLOPE)
              + Constants.HOOD_ANGLE_OFFSET;
          shooter.setSpeed(speed / 100);
          Hood.setToPosition(angle);
          if (Hood.isReady()) {
            autonShootingStartTime = System.currentTimeMillis();
            autonState = AutonState.SHOOT_BALL;
          }
          Drivetrain.setNoRamp(0);
          break;

        case SHOOT_BALL:
          shooter.runFeeder(true);
          shooter.runElevator(Constants.ELEVATOR_FULL_SPEED);
          Intake.runConveyor(true);
          if (System.currentTimeMillis() - autonShootingStartTime >= Constants.AUTON_SHOOTER_WAIT_TIME) {
            shooter.runFeeder(false);
            shooter.runElevator(0);
            Intake.runConveyor(false);
            autonState = AutonState.ALIGN_TO_THIRD_BALL;
          }
          break;

        case ALIGN_TO_THIRD_BALL:
          if (Drivetrain.alignToBall()) {
            Drivetrain.setNoRamp(0);
            Intake.run(INTAKE_MODE.FORWARD);
            autonState = AutonState.GOTO_THIRD_BALL;
          }
          break;

        case GOTO_THIRD_BALL:
          if (Drivetrain.getDanger()){
            autonState = AutonState.WAIT_FOR_BALL;
          }
          if (!beamBreak.get()) {
            Drivetrain.setNoRamp(0);
            // System.out.println(drivetrain.getPosition());
            // drivetrain.setPosition(0);
            // System.out.println(drivetrain.getPosition());
            // drivetrain.setPosition(0);
            autonHumanStationWait = System.currentTimeMillis();
            robotPos = Drivetrain.getPosition();
            autonState = AutonState.WAIT_FOR_BALL;
          } else {
            if (PhotonCamera.getArea() > 5) {
              thirdBallClose = true;
            }
            angle = PhotonCamera.getYaw();
            double slowSpeed = 0.5;
            Drivetrain.runInverseKinematics((Constants.BALL_FOLLOWING_kP * -angle),
                !thirdBallClose ? Constants.AUTON_SPEED_M_PER_S : slowSpeed);
          }
          break;

        case WAIT_FOR_BALL:
          if (System.currentTimeMillis() - autonHumanStationWait > Constants.AUTON_HUMAN_BALL_WAIT_TIME) {
            autonState = AutonState.MOVE_TOWARDS_GOAL;
          } else {
            Drivetrain.setNoRamp(0);
          }
          break;

        case MOVE_TOWARDS_GOAL:
          if (Math.abs(Drivetrain.getPosition() - robotPos) >= Constants.DISTANCE_FROM_HUMAN_STATION) {
            do
            Drivetrain.stop();
            while (!Drivetrain.isReady());
            autonState = AutonState.ALIGN_TO_GOAL_AGAIN;
          } else {
            Drivetrain.runInverseKinematics(0, -Constants.AUTON_SPEED_M_PER_S);
          }
          break;

        case ALIGN_TO_GOAL_AGAIN:
          if (Drivetrain.alignToGoal()) {
            autonState = Constants.AutonState.SHOOT_AGAIN;
          }
          break;

        case SHOOT_AGAIN:
          distance = Limelight.estimateDistance();
          speed = (distance * Constants.HOOD_SPEED_SLOPE) + Constants.HOOD_SPEED_OFFSET;
          angle = (distance * distance * Constants.HOOD_ANGLE_CURVE) + (distance * Constants.HOOD_ANGLE_SLOPE)
              + Constants.HOOD_ANGLE_OFFSET;
          shooter.setSpeed(speed / 100);
          Hood.setToPosition(angle);
          if (Hood.isReady()) {
            autonShootingStartTime = System.currentTimeMillis();
            shooter.runFeeder(true);
            shooter.runElevator(Constants.ELEVATOR_FULL_SPEED);
            Intake.runConveyor(true);
            if (System.currentTimeMillis() - autonShootingStartTime >= Constants.AUTON_SHOOTER_WAIT_TIME) {
              autonState = AutonState.ALIGN_TO_THIRD_BALL;
              shooter.runFeeder(false);
              shooter.runElevator(0);
              Intake.runConveyor(false);
              Limelight.setLedOn(false);
            }
          }
          Drivetrain.setNoRamp(0);
          break;

        case NONE:
          Drivetrain.setNoRamp(0);
          break;
      }
      inUse = false;
    }
  }

  public void simpleAuton() {
    // State machine for auton
    if (inUse == false) {
      inUse = true;
      switch (autonState) {
        // Go until ball finds the starting tape
        case FIND_LINE:
          break;

        // After the line, go to where we know the ball is (~40in outside of the tape)

        case ALIGN_TO_BALL:
          shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
          Hood.home();
          if (Drivetrain.alignToBall()) {
            Drivetrain.setNoRamp(0);
            autonState = AutonState.GOTO_BALL;
          }
          break;

        case GOTO_BALL:
          shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
          Hood.home();
          Limelight.setLedOn(true);
          // Beam break here
          if (!beamBreak.get()) {
            do {
              Drivetrain.setPosition(0);
            } while (Drivetrain.getPosition() > 1);
            autonState = AutonState.TRACK_GOAL;
          }
          double angle = PhotonCamera.getYaw();
          Drivetrain.runInverseKinematics(Constants.BALL_FOLLOWING_kP * -angle, Constants.AUTON_SPEED_M_PER_S * (2.0 / 3));
          break;

        // Turn off intake after the ball is picked up
        case PICKUP_BALL:
          shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
          Hood.home();
          inUse = true;
          System.out.println(Drivetrain.getPosition());
          if (Drivetrain.getPosition() >= Constants.BALL_DISTANCE_FROM_BOT) {
            Drivetrain.setNoRamp(0);
            // intake.run(INTAKE_MODE.OFF);
            autonState = AutonState.TRACK_GOAL;
          } else {
            Drivetrain.set(Constants.AUTON_SPEED);
          }
          break;
        case TRACK_GOAL:
          shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
          Hood.home();
          if (Drivetrain.alignToGoal()) {
            autonState = Constants.AutonState.PREPARE_SHOOTER;
          }
          break;

        case PREPARE_SHOOTER:
          double distance = Limelight.estimateDistance();
          double speed = (distance * Constants.HOOD_SPEED_SLOPE) + Constants.HOOD_SPEED_OFFSET;
          angle = (distance * distance * Constants.HOOD_ANGLE_CURVE) + (distance * Constants.HOOD_ANGLE_SLOPE)
              + Constants.HOOD_ANGLE_OFFSET;
          shooter.setSpeed(speed / 100);
          Hood.setToPosition(angle);
          if (Hood.isReady()) {
            autonShootingStartTime = System.currentTimeMillis();
            autonState = AutonState.SHOOT_BALL;
          }
          Drivetrain.setNoRamp(0);
          break;

        case SHOOT_BALL:
          shooter.runFeeder(true);
          shooter.runElevator(Constants.ELEVATOR_FULL_SPEED);
          Intake.runConveyor(true);
          break;

        case NONE:
          Drivetrain.setNoRamp(0);
          break;
        
        default:
          Drivetrain.set(0);
          break;
      }
      inUse = false;
    }
  }

  public void gatherData() {
    owenGlag = telemetryTable.getEntry("owenGlag").getBoolean(true);
    if (owenGlag) {
      telemetryTable.getEntry("time").setDouble(System.currentTimeMillis() - startTime);
      climber.telemetry(telemetryTable);
      Drivetrain.telemetry(telemetryTable);
      Hood.telemetry(telemetryTable);
      Intake.telemetry(telemetryTable);
      shooter.telemetry(telemetryTable);
      Limelight.telemetry(telemetryTable.getSubTable("limelight"));
      PhotonCamera.telemetry(telemetryTable.getSubTable("photonvision"));

      telemetryTable.getEntry("autonState").setNumber(autonStateToInt());
      telemetryTable.getEntry("started").setBoolean(started);
      telemetryTable.getEntry("disabled").setBoolean(RobotState.isDisabled());
      telemetryTable.getEntry("owenGlag").setBoolean(false);
      telemetryTable.getSubTable("inputs").getEntry("run_feeder").setBoolean(secondDS.getRawButton(Controls.SecondDriverStation.FEED_BALL_TO_SHOOTER));
      telemetryTable.getEntry("battery").setDouble(RobotController.getBatteryVoltage());
      dropCount = 0;
    } else {
      dropCount += 1;
      droppedTelemetryEntry.setDouble(dropCount);
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
