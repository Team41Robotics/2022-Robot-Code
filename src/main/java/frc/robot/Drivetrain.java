package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.RelativeEncoder;




public class Drivetrain {
    private CANSparkMax sparkLF;
    private CANSparkMax sparkLB;
    private CANSparkMax sparkRF;
    private CANSparkMax sparkRB;
    private Joystick leftJoy; 
    private Joystick rightJoy;
    private RelativeEncoder lfEncoder;
    private CANSparkMax[] sparkList = new CANSparkMax[4];

    /** Intialize all sparks, joysticks, and encoder */
    public Drivetrain(){
        sparkLF = new CANSparkMax(15, MotorType.kBrushless);
        sparkRF = new CANSparkMax(5, MotorType.kBrushless);
        sparkLB = new CANSparkMax(1, MotorType.kBrushless);
        sparkRB = new CANSparkMax(4, MotorType.kBrushless);
        lfEncoder = sparkLF.getEncoder();
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
        return lfEncoder.getPosition()*Math.PI/2;
    }

    /** Set position (inches) of the robot */
    public void setPosition(double pos) {
        lfEncoder.setPosition(pos);
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
        double leftSpeed = leftJoy.getY();
        double rightSpeed = rightJoy.getY();
    
        if(Math.abs(leftSpeed) > .1){
            sparkLB.set(leftSpeed/2);
            sparkLF.set(leftSpeed/2);
        } else {
            sparkLB.set(0);
            sparkLF.set(0);
        }

        if(Math.abs(rightSpeed) > .1) {
            sparkRB.set(rightSpeed/2);
            sparkRF.set(rightSpeed/2);
        } else {
            sparkRB.set(0);
            sparkRF.set(0);
        }
    }
    
}
