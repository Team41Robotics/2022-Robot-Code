package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;


public class Drivetrain {
    private CANSparkMax sparkLF;
    private CANSparkMax sparkLB;
    private CANSparkMax sparkRF;
    private CANSparkMax sparkRB;
    private Joystick leftJoy; 
    private Joystick rightJoy;

    public Drivetrain(){
        sparkLF = new CANSparkMax(15, MotorType.kBrushless);
        sparkRF = new CANSparkMax(5, MotorType.kBrushless);
        sparkLB = new CANSparkMax(1, MotorType.kBrushless);
        sparkRB = new CANSparkMax(4, MotorType.kBrushless);

        sparkRB.setInverted(true);
        sparkRF.setInverted(true);
        leftJoy = Robot.leftJoy;
        rightJoy = Robot.rightJoy;
    }

    public void auton(){
        if(!Robot.onTape){
            sparkLB.set(.1);
            sparkLF.set(.1);
            sparkRB.set(.1);
            sparkRF.set(.1);
        }else{
            sparkLB.set(0);
            sparkLF.set(0);
            sparkRB.set(0);
            sparkRF.set(0); 
        }
        

    }
    
    public void teleop(){
        double leftSpeed = leftJoy.getY();
        double rightSpeed = rightJoy.getY();
        sparkLB.set(leftSpeed/2);
        sparkLF.set(leftSpeed/2);
        sparkRB.set(rightSpeed/2);
        sparkRF.set(rightSpeed/2);
    }

}
