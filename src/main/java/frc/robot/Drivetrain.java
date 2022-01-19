ackage frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;




public class Drivetrain {
    private CANSparkMax sparkLF;
    private CANSparkMax sparkLB;
    private CANSparkMax sparkRF;
    private CANSparkMax sparkRB;
    private Joystick leftJoy; 
    private Joystick rightJoy;
    private SparkMaxPIDController controllerLF;
    private SparkMaxPIDController controllerRF;
    private RelativeEncoder lfEncoder;

    public Drivetrain(){
        sparkLF = new CANSparkMax(15, MotorType.kBrushless);
        sparkRF = new CANSparkMax(5, MotorType.kBrushless);
        sparkLB = new CANSparkMax(1, MotorType.kBrushless);
        sparkRB = new CANSparkMax(4, MotorType.kBrushless);
        CANSparkMax[] sparkList = {sparkLF,sparkRF, sparkLB, sparkRB};
        lfEncoder = sparkLF.getEncoder();

        sparkRB.setInverted(true);
        sparkRF.setInverted(true);
        leftJoy = Robot.leftJoy;
        rightJoy = Robot.rightJoy;

        //controllerLF = sparkLF.getPIDController();
        //sparkLB.follow(sparkLF);
       // controllerRF = sparkRF.getPIDController();
       // sparkRB.follow(sparkRF);
        
       // controllerLF.setFF(0.00000481);
        //controllerRF.setFF(0.00000481);

    }

    public void auton(){
        if(!Robot.onTape){
            setAllSparks(0.1);
            System.out.println(lfEncoder.getVelocity()*Math.PI/2);
        }else{
            setAllSparks(0);
 
        }
    }

    public double getPosition() {
        return lfEncoder.getPosition()*Math.PI/2;
    }

    public void setPosition(double pos) {
        lfEncoder.setPosition(pos);
    }

    public void set(double speed) {
        setAllSparks(speed);
    }

    public void stop() {
        setAllSparks(0);
    }
    
    public void teleop(){
        double leftSpeed = leftJoy.getY();
        double rightSpeed = rightJoy.getY();
        sparkLB.set(leftSpeed/2);
        sparkLF.set(leftSpeed/2);
        sparkRB.set(rightSpeed/2);
        sparkRF.set(rightSpeed/2);
        // System.out.println(leftSpeed);
        // System.out.println(rightSpeed);
    }
    private void setAllSparks(double speed){
        for(CANSparkMax i : sparkList){
            i.set(speed);
        }
    }
}
