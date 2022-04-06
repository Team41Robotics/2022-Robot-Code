package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort.Port;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

/** Class for manipulating the robot drivetrain */
public class Drivetrain {
    public static long startTime;
    private static boolean climbing;
    private static double angleToBall;
    private static AHRS navx = new AHRS(Port.kUSB);
    private static Joystick leftJoy;
    private static Joystick rightJoy;
    private static PID leftBackPID;
    private static PID leftFrontPID;
    private static PID rightBackPID;
    private static PID rightFrontPID;
    private static PositionalPID ballTrackingPID;
    private static TalonFX talonLF, talonLB, talonRF, talonRB;
    private static TalonFX[] talonList = new TalonFX[4];
    
    /**
     * Intialize all falcons, their encoders and PID controllers, and joysticks
     */
    public static void initDrivetrain() {
        talonLB = new TalonFX(Constants.FALCON_LB);
        talonLF = new TalonFX(Constants.FALCON_LF);
        talonRF = new TalonFX(Constants.FALCON_RF);
        talonRB = new TalonFX(Constants.FALCON_RB);

        talonList[0] = talonLF;
        talonList[1] = talonLB;
        talonList[2] = talonRF;
        talonList[3] = talonRB;

        talonLB.setInverted(true); 
        talonLF.setInverted(true);
        
        leftJoy = Robot.leftJoy;
        rightJoy = Robot.rightJoy;

        leftBackPID = new PID(talonLB, Constants.kP, Constants.kI, Constants.kD, Constants.kFF, Constants.RAMP_TIME);
        leftFrontPID = new PID(talonLF, Constants.kP, Constants.kI, Constants.kD, Constants.kFF, Constants.RAMP_TIME);
        rightBackPID = new PID(talonRB, Constants.kP, Constants.kI, Constants.kD, Constants.kFF, Constants.RAMP_TIME);
        rightFrontPID = new PID(talonRF, Constants.kP, Constants.kI, Constants.kD, Constants.kFF, Constants.RAMP_TIME);
        
        ballTrackingPID = new PositionalPID(Constants.BALLTRACKING_P);
    }

    /**
     * Get position of the robot
     * @return position of the robot in inches
     */
    public static double getPosition() {
        return talonLF.getSelectedSensorPosition()*Constants.WHEEL_CONVERSION_FACTOR;
    }

    /**
     * Set position of the robot
     * @param pos position of the robot in inches
     */
    public static void setPosition(double pos) {
        for (TalonFX tal : talonList) {
            tal.setSelectedSensorPosition(pos);
        }
    }

    /**
     * Set the speed of the drivetrain
     * @param speed desired speed of the robot [-1, 1]
     */
    public static void set(double speed) {
        leftBackPID.run(speed);
        leftFrontPID.run(speed);
        rightBackPID.run(speed);
        rightFrontPID.run(speed);
    }

    /**
     * Set the speed of the drivetrain without the ramp in PID
     * @param speed desired speed of the robot [-1, 1]
     */
    public static void setNoRamp(double speed) {
        leftBackPID.runNoRamp(speed);
        leftFrontPID.runNoRamp(speed);
        rightBackPID.runNoRamp(speed);
        rightFrontPID.runNoRamp(speed);
    }

    /**
     * Stop the drivetrain
     */
    public static void stop() {
        set(0);
    }
    
    /**
     * Run the drivetrain in teleoperated mode, with both slow and fast modes, as well as small joystick deadzones
     */
    public static void teleop() { 
        double leftSpeed = joystickTransfer(-leftJoy.getY());
        double rightSpeed = joystickTransfer(-rightJoy.getY());
    
        if(Math.abs(leftSpeed) > (climbing ? Constants.JOYSTICK_CLIMBING_MODE_DEADZONE : Constants.JOYSTICK_DEADZONE)) {
            setLeft(leftSpeed);
        } else {
            setLeft(0);
        }

        if(Math.abs(rightSpeed) > (climbing ? Constants.JOYSTICK_CLIMBING_MODE_DEADZONE : Constants.JOYSTICK_DEADZONE)) {
            setRight(rightSpeed);
        } else {
            setRight(0);
        }

        if (rightJoy.getRawButtonPressed(Constants.CLIMBING_DRIVE_BUTTON)) {
            climbing = !climbing;
        }
    }

    /**
     * Sets the speed of the left motors
     * @param speed desired speed [-1, 1]
     */
    public static void setLeft(double speed) {
        leftBackPID.run(speed);
        leftFrontPID.run(speed);
    }

    /**
     * Sets the speed of the right motors
     * @param speed desired speed [-1, 1]
     */
    public static void setRight(double speed) {
        rightBackPID.run(speed);
        rightFrontPID.run(speed);
    }

    /**
     * Sets the speed of the left motors without the ramp from PID
     * @param speed desired speed [-1, 1]
     */
    public static void setLeftNoRamp(double speed) {
        leftBackPID.runNoRamp(speed);
        leftFrontPID.runNoRamp(speed);
    }

    /**
     * Sets the speed of the right motors without the ramp from PID
     * @param speed desired speed [-1, 1]
     */
    public static void setRightNoRamp(double speed) {
        rightBackPID.runNoRamp(speed);
        rightFrontPID.runNoRamp(speed);
    }

    /**
     * Adjusts the orientation of the robot in accordance to its relation with the tape
     * @return Whether the robot is within the alignment threshold of the hub
     */
    public static boolean alignToGoal() {
        double angle = Limelight.getRobotAngle();
        if (Limelight.targetFound() && angle>Constants.ALIGNMENT_HORIZONTAL_THRESHHOLD) {
            setRight(-Constants.AUTON_SPEED/2);
            setLeft(Constants.AUTON_SPEED/2);
        } else if (Limelight.targetFound() && angle<-Constants.ALIGNMENT_HORIZONTAL_THRESHHOLD) {
            setRight(Constants.AUTON_SPEED/2);
            setLeft(-Constants.AUTON_SPEED/2);
        } else {
            setNoRamp(0);
            return true;
        }
        return false;
    }

    /**
     * Prepare the robot to align to a ball
     */
    public static void setupAlignmentToBall() {
        if (!PhotonCamera.hasTarget()) {
            setupAlignmentToBall();
        } else {
            angleToBall = PhotonCamera.getYaw();
            navx.zeroYaw();
        }
    }

    /**
     * Align the robot to the nearest ball of the correct color
     * @return Whether the robot is within the alignment threshold of the ball
     */
    public static boolean alignToBall() {
        double angle = navx.getAngle();
        double speed = ballTrackingPID.run(angle);
        if (speed < Constants.AUTON_SPEED/2) speed = Constants.AUTON_SPEED/2;
        if (speed > -Constants.AUTON_SPEED/2) speed = -Constants.AUTON_SPEED/2;
        if (angleToBall - angle>Constants.ALIGNMENT_HORIZONTAL_THRESHHOLD) {
            setRightNoRamp(speed);
            setLeftNoRamp(-speed);
        } else if (angleToBall - angle<-Constants.ALIGNMENT_HORIZONTAL_THRESHHOLD) {
            setRightNoRamp(-speed);
            setLeftNoRamp(speed);
        } else {
            setNoRamp(0);
            return true;
        }
        return false;
    }
    
    /**
     * Run the drivetrain using inverse kinematics
     * @param angularVel the desired angular velocity of the robot (rad/s)
     * @param linearVel the desired linear velocity of the robot (m/s)
     */
    public static void runInverseKinematics(double angularVel, double linearVel) {
        double leftWheelAngularVelocity;
        double rightWheelAngularVelocity;

        rightWheelAngularVelocity = (1/Constants.WHEEL_RAD)*(linearVel + angularVel*Constants.ROBOT_DIAMETER);
        leftWheelAngularVelocity = (1/Constants.WHEEL_RAD)*(linearVel - angularVel*Constants.ROBOT_DIAMETER);

        rightWheelAngularVelocity *= Constants.WHEEL_RADPERSEC_TO_MOTOR_RPM;
        leftWheelAngularVelocity *= Constants.WHEEL_RADPERSEC_TO_MOTOR_RPM;

        setLeft(leftWheelAngularVelocity/Constants.MAX_RPM);
        setRight(rightWheelAngularVelocity/Constants.MAX_RPM);
    }

    /**
     * Function to convert joystick input to a function (basically joystick acceleration)
     * @param joyVal the inputted value from the joystick
     * @return the adjusted value
     */
    public static double joystickTransfer(double joyVal) {
        if (climbing) {
            double newJoyVal = Math.pow(joyVal, 2);
            newJoyVal *= Constants.CLIMBING_DRIVE_MAX_SPEED;
            newJoyVal += Constants.CLIMBING_DRIVING_SPEED_OFFSET;
            if (joyVal > 0) {
                return newJoyVal;
            } else {
                return -newJoyVal;
            } 
        } else {
            return joyVal * Constants.DRIVETRAIN_MAX_SPEED;
        }
    }

    /**
     * A function to test any features of the drivetrain
     */
    public static void test() {
        if (System.currentTimeMillis() - startTime <= 2000) {
            set(0.1);
        } else {
            set(0);
        }
    }

    /**
     * Output all necessary telemetry data from the drivetrain
     * @param table the base telemetry NetworkTable
     */
    public static void telemetry(NetworkTable table) {
        NetworkTable motorTable = table.getSubTable("motors");

        leftFrontPID.telemetry(motorTable, "Left Front Drivetrain Motor");
        leftBackPID.telemetry(motorTable, "Left Back Drivetrain Motor");
        rightFrontPID.telemetry(motorTable, "Right Front Drivetrain Motor");
        rightBackPID.telemetry(motorTable, "Right Back Drivetrain Motor");
    }

    /**
     * Get whether or not the drivetrain is at the desired speed
     * @return If the drivetrain is at the desired speed
     */
    public static boolean isReady() {
        return leftFrontPID.isReady();
    }

    /**
     * Get if the drivetrain current is too high
     * @return true if any of the drivetrain motors are reading more than 80 amps
     */
    public static boolean getDanger(){
        return (leftBackPID.getCurrent()>80) || (leftFrontPID.getCurrent()>80)|| (rightBackPID.getCurrent()>80) || (rightFrontPID.getCurrent()>80);
    }
}
