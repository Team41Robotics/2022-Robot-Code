package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
/** Class for manipulating the robot drivetrain */
public class Drivetrain {
    private boolean climbing;
    private Joystick leftJoy;
    private Joystick rightJoy;
    private TalonFX talonLF, talonLB, talonRF, talonRB;
    private TalonFX[] talonList = new TalonFX[4];
    private PID leftBackPID;
    private PID leftFrontPID;
    private PID rightBackPID;
    private PID rightFrontPID;
    
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

        leftBackPID = new PID(talonLB, 0.6, 0.25, 0.000275, 1.1, 0);
        leftFrontPID = new PID(talonLF, 0.6, 0.25, 0.000275, 1.1, 0);
        rightBackPID = new PID(talonRB, 0.6, 0.25, 0.000275, 1.1, 0);
        rightFrontPID = new PID(talonRF, 0.6, 0.25, 0.000275, 1.1, 0);
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

        if (rightJoy.getRawButtonPressed(2)) {
            climbing = !climbing;
        }
        System.out.println(climbing ? "Climbing" : "Normal");
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
    public void auton() {
        double angle = Limelight.getHorizontalAngle();
        if (angle>Constants.LIMELIGHT_HORIZONTAL_THRESHHOLD) {
            setRight(-Constants.AUTON_SPEED/2);
            setLeft(Constants.AUTON_SPEED/2);
        } else if (angle<-Constants.LIMELIGHT_HORIZONTAL_THRESHHOLD) {
            setRight(Constants.AUTON_SPEED/2);
            setLeft(-Constants.AUTON_SPEED/2);
        } else {
            set(0);
            System.out.println("Buffer Moment");
        }
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
            newJoyVal *= Constants.CLIMBING_MAX_SPEED;
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
}
