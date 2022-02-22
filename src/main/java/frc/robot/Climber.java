package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
    private CANSparkMax climbingMotor1;
    private CANSparkMax climbingMotor2;
    private Joystick leftJoy;
    private Joystick driverStation;
    private DigitalInput firstStageLeftSwitch, firstStageRightSwitch, secondStageSwitch;
    private boolean firstStageUp;
    private DoubleSolenoid secondStageGearLock, firstStageGearLock, secondStageRelease , gearShifter; // secondStageRelease is second stage piston

    public Climber() {
        climbingMotor1 = new CANSparkMax(Constants.CLIMBING_SPARK_F, MotorType.kBrushless);
        climbingMotor2 = new CANSparkMax(Constants.CLIMBING_SPARK_B, MotorType.kBrushless);
        climbingMotor1.setIdleMode(IdleMode.kBrake);
        climbingMotor2.setIdleMode(IdleMode.kBrake);
        
        secondStageGearLock = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.OUTER_GEAR_LOCK_ON, Constants.OUTER_GEAR_LOCK_OFF);
        firstStageGearLock = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.INNER_GEAR_LOCK_ON, Constants.INNER_GEAR_LOCK_OFF);
        secondStageRelease = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.MIDDLE_ARM_RELEASE, Constants.MIDDLE_ARM_LOCK);
        gearShifter = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.MOVE_TO_INNER_ARM, Constants.MOVE_TO_OUTER_ARMS);

        this.driverStation = new Joystick(3);
        this.leftJoy = Robot.leftJoy;

        firstStageUp = false;
        secondStageGearLock.set(DoubleSolenoid.Value.kReverse);
        secondStageRelease.set(DoubleSolenoid.Value.kReverse);
        firstStageGearLock.set(DoubleSolenoid.Value.kOff);
        gearShifter.get();  

        firstStageLeftSwitch = new DigitalInput(Constants.FIRST_STAGE_LIMIT_SWITCH_L);
        firstStageRightSwitch = new DigitalInput(Constants.FIRST_STAGE_LIMIT_SWITCH_R);
    }

    public void moveIntake(double speed) {
        climbingMotor1.set(speed);
        climbingMotor2.set(speed);
        SmartDashboard.putNumber("Motor 1 set speed", climbingMotor1.get());
    }

    public void teleop() {
        switch((int) (this.getPOV())) {
        case(0):
            if (!firstStageUp) {
                climbingMotor1.set(-Constants.CLIMBING_SPEED);
                climbingMotor2.set(-Constants.CLIMBING_SPEED);
                if (firstStageLeftSwitch.get() || firstStageRightSwitch.get()) {
                    firstStageUp = true;
                }
            }
            break;
        case(45):
            firstStageGearLock.set(DoubleSolenoid.Value.kForward);
            secondStageGearLock.set(DoubleSolenoid.Value.kForward);
            break;
        case(90):
            
            // send out third arm
            // i have no idea what the third arm is there arent any labels :(
            break;
        case(135):
            // disable piston for third arm
            
            break;
        case(180):
            // pull in third arm
            break;
        case(225):
            // hold in place
            break;
        case(270):
            break;
        case(315):
            break;
        }
    }

    public double getPOV(){
        return driverStation.getDirectionDegrees();
    }

   

}
