// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.AutonState;

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
    public static Joystick leftJoy = new Joystick(Constants.LEFT_JOY);
    public static Joystick rightJoy = new Joystick(Constants.RIGHT_JOY);
    public static Joystick secondDS = new Joystick(Constants.RIGHT_DRIVER_STATION);
    private boolean owenGlag, started;
    private int dropCount;
    private long startTime;
    private AutonState autonState;
    private DigitalInput beamBreak;
    private NetworkTable telemetryTable;
    private SendableChooser<Integer> autonChooser;
    
    /**
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */
    @Override
    public void robotInit() {
        started = false;
        telemetryTable = NetworkTableInstance.getDefault().getTable("telemetry");
        startTime = System.currentTimeMillis();
        beamBreak = new DigitalInput(Constants.BEAM_BREAK_PORT);
        autonState = AutonState.NONE;

        SmartDashboard.putNumber("LL Distance", -1);
        SmartDashboard.putNumber("Speed Offset", Constants.HOOD_SPEED_OFFSET);
        SmartDashboard.putBoolean("Shooter Ready", false);
        SmartDashboard.putBoolean("Beam Break", false);
        SmartDashboard.putNumber("Telemetry Packets Dropped", -1);
        
        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("4 Ball Auton", 3);
        autonChooser.addOption("2 Ball Auton", 2);
        autonChooser.addOption("Simple Auton", 1);
        autonChooser.addOption("No Auton", 0);
        SmartDashboard.putData("Do Real Auton", autonChooser);

        Shooter.initShooter();
        Hood.initHood();
        Climber.initClimber();
        Drivetrain.initDrivetrain();
        Intake.initIntake();
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
        telemetryTable.getEntry("disabled").setBoolean(false);
        started = true;
        autonState = AutonState.ALIGN_TO_BALL;
        Auton.init(beamBreak);
        Drivetrain.setPosition(0);
        Drivetrain.setupAlignmentToBall();
        Drivetrain.stop();
        PhotonCamera.setPipeline(DriverStation.getAlliance() == Alliance.Blue);
        Hood.home();
        Intake.autonInit();
        Limelight.resetZoom();
        Shooter.setSpeed(0);
    }
    
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        switch (autonChooser.getSelected()) {
            case 0:
                // do nothing
                break;
            case 1:
                // move forward
                break;
            case 2:
                // 2 ball
                simpleAuton();
                break;
            case 3:
                // 4 ball
                fullAuton();
                break;
        } 
    }
    
    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        autonState = AutonState.NONE;
        telemetryTable.getEntry("disabled").setBoolean(false);
        Climber.reset();
        Hood.home();
        Intake.reset();
        Limelight.setLedOn(false);
        Limelight.resetZoom();
        Shooter.setSpeed(0);
    }
    
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        Intake.teleop();
        Climber.teleop();
        Shooter.teleop();
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
            Shooter.setSpeed(0);
        } else if (secondDS.getRawButton(Controls.SecondDriverStation.LOW_GOAL_SETUP)) {
            Limelight.setLedOn(false);
            Shooter.setSpeed(Constants.LOW_GOAL_SPEED);
            Hood.setToPosition(Constants.LOW_GOAL_ANGLE);
        } else if (secondDS.getRawButton(Controls.SecondDriverStation.AUTO_SHOOTING)) {
            Limelight.setLedOn(true);
            if (Limelight.targetFound()) {
                Shooter.setSpeed(speed / 100);
                Hood.setToPosition(angle);
                Drivetrain.alignToGoal();
            }
        } else if (secondDS.getRawButton(Controls.SecondDriverStation.SHOOTER_WARMUP)) {
            Limelight.setLedOn(true);
            Shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
            Hood.setToPosition(Constants.HOOD_DEFAULT_ANGLE);
            Drivetrain.teleop();
        } else if (Climber.climbing) {
            Shooter.setSpeed(0);
            Limelight.setLedOn(false);
            Drivetrain.teleop();
        } else {
            Drivetrain.teleop();
            Hood.teleop();
            Shooter.setSpeed(0);
            Limelight.setLedOn(false);
        }
        
        SmartDashboard.putNumber("LL Distance", Limelight.estimateDistance());
        SmartDashboard.putNumber("Speed Offset", Constants.HOOD_SPEED_OFFSET);
        SmartDashboard.putBoolean("Shooter Ready", Shooter.isReady());
        SmartDashboard.putBoolean("Beam Break", !beamBreak.get());
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
        PhotonCamera.setPipeline(false);
    }
    
    public void fullAuton() {
        switch (autonState) {
            case ALIGN_TO_BALL:
            autonState = Auton.alignToBall();
            break;
            
            case GOTO_BALL:
            autonState = Auton.gotoBall();
            break;
            
            case PICKUP_BALL:
            autonState = AutonState.NONE;
            break;
            
            case TRACK_GOAL:
            autonState = Auton.trackGoal();
            break;
            
            case PREPARE_SHOOTER:
            autonState = Auton.prepareShooter();
            break;
            
            case SHOOT_BALL:
            autonState = Auton.shootBall();
            break;
            
            case ALIGN_TO_THIRD_BALL:
            autonState = Auton.alignToThirdBall();
            break;
            
            case GOTO_THIRD_BALL:
            autonState = Auton.gotoThirdBall();
            break;
            
            case WAIT_FOR_BALL:
            autonState = Auton.waitForBall();
            break;
            
            case MOVE_TOWARDS_GOAL:
            autonState = Auton.moveTowardsGoal();
            break;
            
            case ALIGN_TO_GOAL_AGAIN:
            autonState = Auton.alignToGoalAgain();
            break;
            
            case SHOOT_AGAIN:
            Auton.shootAgain();
            break;
            
            case NONE:
            Auton.none();
            break;
        }
    }
    
    public void simpleAuton() {
        switch (autonState) {
            case ALIGN_TO_BALL:
            autonState = Auton.alignToBall();
            break;
            
            case GOTO_BALL:
            autonState = Auton.gotoBall();
            break;
            
            case PICKUP_BALL:
            autonState = AutonState.NONE;
            break;
            
            case TRACK_GOAL:
            autonState = Auton.trackGoal();
            break;
            
            case PREPARE_SHOOTER:
            autonState = Auton.prepareShooter();
            break;
            
            case SHOOT_BALL:
            autonState = Auton.shootBall();
            break;
            
            case NONE:
            Auton.none();
            break;
            
            default:
            Drivetrain.set(0);
            break;
        }
    }
    
    public void superSimpleAuton() {
        if (autonState == AutonState.MOVE_TOWARDS_GOAL) {
            autonState = Auton.moveForward();
        } else {
            Drivetrain.stop();
        }
    }

    public void gatherData() {
        owenGlag = telemetryTable.getEntry("owenGlag").getBoolean(true);
        if (owenGlag) {
            telemetryTable.getEntry("time").setDouble(System.currentTimeMillis() - startTime);
            Climber.telemetry(telemetryTable);
            Drivetrain.telemetry(telemetryTable);
            Hood.telemetry(telemetryTable);
            Intake.telemetry(telemetryTable);
            Shooter.telemetry(telemetryTable);
            Limelight.telemetry(telemetryTable.getSubTable("limelight"));
            PhotonCamera.telemetry(telemetryTable.getSubTable("photonvision"));
            
            telemetryTable.getEntry("autonState").setNumber(autonStateToInt());
            telemetryTable.getEntry("started").setBoolean(started);
            telemetryTable.getEntry("disabled").setBoolean(RobotState.isDisabled());
            telemetryTable.getEntry("owenGlag").setBoolean(false);
            telemetryTable.getSubTable("inputs").getEntry("run_feeder").setBoolean(secondDS.getRawButton(Controls.SecondDriverStation.FEED_BALL_TO_SHOOTER));
            telemetryTable.getEntry("battery").setDouble(RobotController.getBatteryVoltage());
            telemetryTable.getEntry("match").setDouble(DriverStation.getMatchNumber());
            telemetryTable.getEntry("event").setString(DriverStation.getEventName());
            dropCount = 0;
        } else {
            dropCount += 1;
            SmartDashboard.putNumber("Telemetry Packets Dropped", dropCount);
        }
    }
    
    public int autonStateToInt() {
        switch (autonState) {
            case NONE:
            return 0;
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
