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
    private CANSparkMax[] sparkList = {sparkLF,sparkRF, sparkLB, sparkRB};

    public Drivetrain(){
        sparkLF = new CANSparkMax(15, MotorType.kBrushless);
        sparkRF = new CANSparkMax(5, MotorType.kBrushless);
        sparkLB = new CANSparkMax(1, MotorType.kBrushless);
        sparkRB = new CANSparkMax(4, MotorType.kBrushless);
        lfEncoder = sparkLF.getEncoder();

        sparkRB.setInverted(true);
        sparkRF.setInverted(true);
        leftJoy = Robot.leftJoy;
        rightJoy = Robot.rightJoy;
    }

    public void auton(){
        if(!Robot.onTape){
            set(0.1);
            System.out.println(lfEncoder.getVelocity()*Math.PI/2);
        }else{
            set(0);
 
        }
    }

    public double getPosition() {
        return lfEncoder.getPosition()*Math.PI/2;
    }

    public void setPosition(double pos) {
        lfEncoder.setPosition(pos);
    }

    public void set(double speed) {
        for(CANSparkMax i : sparkList){
            i.set(speed);
        }
    }

    public void stop() {
        set(0);
    }
    
    public void teleop(){ 
        double leftSpeed = leftJoy.getY();
        double rightSpeed = rightJoy.getY();
       
        if(Math.abs(leftSpeed) <= .05 || Math.abs(rightSpeed) <= .05){
            return;
        }
       
        sparkLB.set(leftSpeed/2);
        sparkLF.set(leftSpeed/2);
        sparkRB.set(rightSpeed/2);
        sparkRF.set(rightSpeed/2);
        // System.out.println(leftSpeed);
        // System.out.println(rightSpeed);
    }
    
}
