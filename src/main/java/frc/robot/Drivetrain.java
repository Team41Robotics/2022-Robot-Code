package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;


public class Drivetrain {
    
    private CANSparkMax sparkLF;
    private CANSparkMax sparkLB;
    private CANSparkMax sparkRF;
    private CANSparkMax sparkRB;
    private Joystick leftJoy; 
    private Joystick rightJoy;  
    private RelativeEncoder lfEncoder;
    private RelativeEncoder rfEncoder;
    private CANSparkMax[] sparkList = new CANSparkMax[4];

    private double leftSpeed;
    private double rightSpeed;

    private double MAX_SPEED;
    private SparkMaxPIDController rightController;
    private SparkMaxPIDController leftController;
    
	

    /** Intialize all sparks, joysticks, and encoder */
    public Drivetrain(){
        sparkLF = new CANSparkMax(Constants.SPARK_LF, MotorType.kBrushless);
        sparkRF = new CANSparkMax(Constants.SPARK_RF, MotorType.kBrushless);
        sparkLB = new CANSparkMax(Constants.SPARK_LB, MotorType.kBrushless);
        sparkRB = new CANSparkMax(Constants.SPARK_RB, MotorType.kBrushless);
        lfEncoder = sparkLF.getEncoder();
        rfEncoder = sparkRF.getEncoder();

        sparkList[0] = sparkLF;
        sparkList[1] = sparkLB;
        sparkList[2] = sparkRF;
        sparkList[3] = sparkRB;
        
        sparkRB.setInverted(true); 
        sparkRF.setInverted(true);
        
        leftJoy = Robot.leftJoy;
        rightJoy = Robot.rightJoy;
    }

    /** Get position (inches) of the robot */
    public double getPosition() {
        return lfEncoder.getPosition()*Constants.WHEEL_CONVERSION_FACTOR;
    }

    /** Set position (inches) of the robot */
    public void setPosition(double pos) {
        lfEncoder.setPosition(pos);
        rfEncoder.setPosition(pos);
    }

    /** Set the speed of the drivetrain [-1, 1] */
    public void set(double speed) {
        for(CANSparkMax i : sparkList){
            i.set(speed);
        }
    }

    /** Stop the drivetrain */
    public void stop() {
        set(0);
    }
    
    /** Run the drivetrain at half the speed of the joysticks */
    public void teleop(){ 
        double leftSpeed = leftJoy.getY()/2;
        double rightSpeed = rightJoy.getY()/2;
    
        if(Math.abs(leftSpeed) > .1){
            sparkLB.set(leftSpeed);
            sparkLF.set(leftSpeed);
        } else {
            sparkLB.set(0);
            sparkLF.set(0);
        }

        if(Math.abs(rightSpeed) > .1) {
            sparkRB.set(rightSpeed);
            sparkRF.set(rightSpeed);
        } else {
            sparkRB.set(0);
            sparkRF.set(0);
        }
    }
    public void setLeft(double speed){
        sparkLF.set(speed);
        sparkLB.set(speed);
    }
    public void setRight(double speed){
        sparkRF.set(speed);
        sparkRB.set(speed);
    }
    
    public void calculateSpeed(){
      
    }

    public void auton() {

    }

}
