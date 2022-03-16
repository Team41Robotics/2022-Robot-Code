package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.photonvision.PhotonCamera;
/** Class for manipulating the robot drivetrain */
public class Drivetrain {
    public boolean aligningToGoal, aligningToBall;
    public long startTime;
    private boolean climbing;
    private Joystick leftJoy;
    private Joystick rightJoy;
    private PhotonCamera driverCam;
    private PID leftBackPID;
    private PID leftFrontPID;
    private PID rightBackPID;
    private PID rightFrontPID;
    private TalonFX talonLF, talonLB, talonRF, talonRB;
    private TalonFX[] talonList = new TalonFX[4];
    
    /** Intialize all sparks, joysticks, and encoder */
    public Drivetrain() {
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
        driverCam = Robot.driverCam;

        leftBackPID = new PID(talonLB, Constants.kP, Constants.kI, Constants.kD, Constants.kFF, Constants.RAMP_TIME);
        leftFrontPID = new PID(talonLF, Constants.kP, Constants.kI, Constants.kD, Constants.kFF, Constants.RAMP_TIME);
        rightBackPID = new PID(talonRB, Constants.kP, Constants.kI, Constants.kD, Constants.kFF, Constants.RAMP_TIME);
        rightFrontPID = new PID(talonRF, Constants.kP, Constants.kI, Constants.kD, Constants.kFF, Constants.RAMP_TIME);
    }

    /**
     * Get position of the robot
     * @return position of the robot in inches
     */
    public double getPosition() {
        return talonLF.getSelectedSensorPosition()*Constants.WHEEL_CONVERSION_FACTOR;
    }

    /**
     * Set position of the robot
     * @param pos position of the robot in inches
     */
    public void setPosition(double pos) {
        for (TalonFX tal : talonList) {
            tal.setSelectedSensorPosition(pos);
        }
    }

    /**
     * Set the speed of the drivetrain
     * @param speed desired speed of the robot [-1, 1]
     */
    public void set(double speed) {
        leftBackPID.run(speed);
        leftFrontPID.run(speed);
        rightBackPID.run(speed);
        rightFrontPID.run(speed);

        SmartDashboard.putNumber("err", leftBackPID.getError());
        SmartDashboard.putNumber("vel", leftBackPID.getVelocity());
        SmartDashboard.putNumber("ctrl", leftBackPID.getControlSignal());
        SmartDashboard.putNumber("current", talonLB.getStatorCurrent());
    }

    public void setNoRamp(double speed) {
        leftBackPID.runNoRamp(speed);
        leftFrontPID.runNoRamp(speed);
        rightBackPID.runNoRamp(speed);
        rightFrontPID.runNoRamp(speed);
        
        SmartDashboard.putNumber("err", leftBackPID.getError());
        SmartDashboard.putNumber("vel", leftBackPID.getVelocity());
        SmartDashboard.putNumber("ctrl", leftBackPID.getControlSignal());
        SmartDashboard.putNumber("current", talonLB.getStatorCurrent());
    }

    /**
     * Stop the drivetrain
     */
    public void stop() {
        set(0);
    }
    
    /** Run the drivetrain at half the speed of the joysticks */
    public void teleop() { 
        double leftSpeed = joystickTransfer(-leftJoy.getY());
        double rightSpeed = joystickTransfer(-rightJoy.getY());
    
        if(Math.abs(leftSpeed) > (climbing ? 0.001 : 0.1)) {
            setLeft(leftSpeed);
        } else {
            setLeft(0);
        }

        if(Math.abs(rightSpeed) > (climbing ? 0.001 : 0.1)) {
            setRight(rightSpeed);
        } else {
            setRight(0);
        }

        if (rightJoy.getRawButtonPressed(Constants.CLIMBING_DRIVE_BUTTON)) {
            climbing = !climbing;
        }
        // System.out.println(climbing ? "Climbing" : "Normal");
        
        // SmartDashboard.putNumber("Motor Speed", leftBackPID.getVelocity());
        // SmartDashboard.putNumber("Motor Ctrl", leftBackPID.getControlSignal());
        // SmartDashboard.putNumber("Left JS Input", leftSpeed);
        // SmartDashboard.putNumber("Right JS Input", rightSpeed);
    }

    /**
     * sets the speed of the left motors
     * @param speed desired speed
     */
    public void setLeft(double speed) {
        leftBackPID.run(speed);
        leftFrontPID.run(speed);
    }

    /**
     * sets the speed of the right motors
     * @param speed desired speed
     */
    public void setRight(double speed) {
        rightBackPID.run(speed);
        rightFrontPID.run(speed);
    }

    /** Adjusts the orientation of the robot in accordance to its relation with the tape */
    public boolean alignToGoal() {
        aligningToGoal = true;
        double angle = Limelight.getRobotAngle();
        if (Limelight.targetFound() && angle>Constants.ALIGNMENT_HORIZONTAL_THRESHHOLD) {
            setRight(-Constants.AUTON_SPEED/2);
            setLeft(Constants.AUTON_SPEED/2);
        } else if (Limelight.targetFound() && angle<-Constants.ALIGNMENT_HORIZONTAL_THRESHHOLD) {
            setRight(Constants.AUTON_SPEED/2);
            setLeft(-Constants.AUTON_SPEED/2);
        } else {
            setNoRamp(0);
            aligningToGoal = false;
            return true;
        }
        return false;
    }

    public boolean alignToBall() {
        aligningToBall = true;
        if (!driverCam.getLatestResult().hasTargets()) {
            return false;
        }
        double angle = driverCam.getLatestResult().getTargets().get(0).getYaw();
        if (angle>Constants.ALIGNMENT_HORIZONTAL_THRESHHOLD) {
            setRight(-Constants.AUTON_SPEED/2);
            setLeft(Constants.AUTON_SPEED/2);
        } else if (angle<-Constants.ALIGNMENT_HORIZONTAL_THRESHHOLD) {
            setRight(Constants.AUTON_SPEED/2);
            setLeft(-Constants.AUTON_SPEED/2);
        } else {
            setNoRamp(0);
            aligningToBall = false;
            return true;
        }
        return false;
    }
    
    // max rpm: 6380 
    public void runInverseKinematics(double angularVel, double linearVel) {
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
     * @param joy the Joystick object that will be transferred
     * @return the adjusted value
     */
    public double joystickTransfer(double joyVal) {
        /** Cubic */
        // joyVal = Math.pow(joyVal, Constants.JOYSTICK_CURVE_POWER);
        // joyVal *= Constants.DRIVETRAIN_MAX_SPEED;
        // return joyVal;

        /** Quadratic */
        // double newJoyVal = Math.pow(joyVal, 2);
        // newJoyVal *= Constants.DRIVETRAIN_MAX_SPEED;
        // if (joyVal > 0) {
        //     return newJoyVal;
        // } else {
        //     return -newJoyVal;
        // }

        /** Linear */
        // return joyVal*.85;

        if (climbing) {
            double newJoyVal = Math.pow(joyVal, 2);
            newJoyVal *= Constants.CLIMBING_DRIVE_MAX_SPEED;
            newJoyVal += 4e-4;
            if (joyVal > 0) {
                return newJoyVal;
            } else {
                return -newJoyVal;
            } 
        } else {
            return joyVal * Constants.DRIVETRAIN_MAX_SPEED;
        }
    }

    public void test() {
        if (System.currentTimeMillis() - startTime <= 2000) {
            set(0.1);
        } else {
            set(0);
        }
        SmartDashboard.putNumber("Motor Speed", leftBackPID.getVelocity());
    }

    public void telemetry(NetworkTable table) {
        NetworkTable motorTable = table.getSubTable("motors");

        leftFrontPID.telemetry(motorTable, "Left Front Drivetrain Motor");
        leftBackPID.telemetry(motorTable, "Left Back Drivetrain Motor");
        rightFrontPID.telemetry(motorTable, "Right Front Drivetrain Motor");
        rightBackPID.telemetry(motorTable, "Right Back Drivetrain Motor");
    }
}
