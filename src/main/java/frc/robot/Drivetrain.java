package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Class for manipulating the robot drivetrain */
public class Drivetrain {
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

        leftBackPID = new PID(talonLB, 0.7, 0.00002, 0.004, 1.2, 2);
        leftFrontPID = new PID(talonLF, 0.7, 0.00002, 0.004, 1.2, 2);
        rightBackPID = new PID(talonRB, 0.7, 0.00002, 0.004, 1.2, 2);
        rightFrontPID = new PID(talonRF, 0.7, 0.00002, 0.004, 1.2, 2);
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
        double leftSpeed = leftJoy.getY()/2;
        double rightSpeed = rightJoy.getY()/2;
    
        if(Math.abs(leftSpeed) > .1) {
            setLeft(leftSpeed);
        } else {
            setLeft(0);
        }

        if(Math.abs(rightSpeed) > .1) {
            setRight(rightSpeed);
        } else {
            setRight(0);
        }
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
        // double angle = Limelight.getHorizontalAngle();
        // if (angle>Constants.LIMELIGHT_HORIZONTAL_THRESHHOLD) {
        //     setRight(-Constants.AUTON_SPEED/2);
        //     setLeft(Constants.AUTON_SPEED/2);
        // } else if (angle<-Constants.LIMELIGHT_HORIZONTAL_THRESHHOLD) {
        //     setRight(Constants.AUTON_SPEED/2);
        //     setLeft(-Constants.AUTON_SPEED/2);
        // } else {
        //     set(0);
        //     System.out.println("Buffer Moment");
        // }
        set(.16);

        // System.out.print(leftFrontPID.getVelocity());
        // System.out.print(", ");
        // System.out.print(leftBackPID.getVelocity());
        // System.out.print(", ");
        // System.out.print(rightFrontPID.getVelocity());
        // System.out.print(", ");
        // System.out.println(rightBackPID.getVelocity());
    }
}
