package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Class for manipulating the robot drivetrain */
public class Drivetrain {
    private Joystick leftJoy;
    private Joystick rightJoy;
    private TalonFX talonLF, talonLB, talonRF, talonRB;
    private TalonFX[] talonList = new TalonFX[4];
    
    /** Intialize all sparks, joysticks, and encoder */
    public Drivetrain() {
        talonLB = new TalonFX(Constants.TALON_LB);
        talonLF = new TalonFX(Constants.TALON_LF);
        talonRF = new TalonFX(Constants.TALON_RF);
        talonRB = new TalonFX(Constants.TALON_RB);

        talonList[0] = talonLF;
        talonList[1] = talonLB;
        talonList[2] = talonRF;
        talonList[3] = talonRB;

        talonRB.setInverted(true); 
        talonRF.setInverted(true);
        
        leftJoy = Robot.leftJoy;
        rightJoy = Robot.rightJoy;
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
        for(TalonFX i : talonList){
            i.set(TalonFXControlMode.PercentOutput, speed);
        }
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
        talonLF.set(TalonFXControlMode.PercentOutput, speed);
        talonLB.set(TalonFXControlMode.PercentOutput, speed);
    }

    /**
     * sets the speed of the right motors
     * @param speed desired speed
     */
    public void setRight(double speed) {
        talonRF.set(TalonFXControlMode.PercentOutput, speed);
        talonRB.set(TalonFXControlMode.PercentOutput, speed);
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
}
