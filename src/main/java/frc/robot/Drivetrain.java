package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.RelativeEncoder;

public class Drivetrain {
    private CANSparkMax sparkLF, sparkLB, sparkRF, sparkRB;
    private Joystick leftJoy;
    private Joystick rightJoy;
    private RelativeEncoder lfEncoder;
    private RelativeEncoder rfEncoder;
    private CANSparkMax[] sparkList = new CANSparkMax[4];
    
	

    /** Intialize all sparks, joysticks, and encoder */
    public Drivetrain() {
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
        sparkLF.set(speed);
        sparkLB.set(speed);
    }
    /**
     * sets the speed of the right motors
     * @param speed desired speed
     */
    public void setRight(double speed) {
        sparkRF.set(speed);
        sparkRB.set(speed);
    }
    
    public void calculateSpeed() {
      
    }
    /**function that adjusts the orientation of the robot in accordance to its relation  with the tape with */
    
    public void auton() {
        double angle = Limelight.getHorizontalAngle();
        if (angle>Constants.LIME_LITE_THRESH_HOLD) {
            setRight(-Constants.AUTON_SPEED);
            setLeft(Constants.AUTON_SPEED);
        } else if (angle<-Constants.LIME_LITE_THRESH_HOLD) {
            setRight(Constants.AUTON_SPEED);
            setLeft(-Constants.AUTON_SPEED);
        } else {
            set(0);
            System.out.println("Buffer Moment");
        }
    }
}
