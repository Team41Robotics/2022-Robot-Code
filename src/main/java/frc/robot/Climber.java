package frc.robot;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Pistons:
 *      secondStageGearLock: fwd - locked; rev - unlocked
 *      firstStageGearLock: fwd - locked; rev - unlocked
 *      secondStageRelease: fwd - locked; rev - unlocked
 *      gearShifter: fwd - first stage; rev - second stage
 */


public class Climber {
    private CANSparkMax climbingMotor1;
    private CANSparkMax climbingMotor2;
    private Joystick leftJoy;
    private Joystick driverStation;
    private DigitalInput firstStageLeftSwitch, firstStageRightSwitch, secondStageSwitch;
    private boolean firstStageUp;
    private boolean secondStageUp;
    private long startTime;
    private DoubleSolenoid secondStageGearLock, firstStageGearLock, secondStageRelease , gearShifter; // secondStageRelease is second stage piston

    public Climber() {
        climbingMotor1 = new CANSparkMax(Constants.CLIMBING_SPARK_F, MotorType.kBrushless);
        climbingMotor2 = new CANSparkMax(Constants.CLIMBING_SPARK_B, MotorType.kBrushless);
        climbingMotor1.setIdleMode(IdleMode.kBrake);
        climbingMotor2.setIdleMode(IdleMode.kBrake);
        
        secondStageGearLock = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.INNER_GEAR_LOCK_OFF, Constants.INNER_GEAR_LOCK_ON);
        firstStageGearLock = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.OUTER_GEAR_LOCK_OFF, Constants.OUTER_GEAR_LOCK_ON);
        secondStageRelease = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.MIDDLE_ARM_LOCK, Constants.MIDDLE_ARM_RELEASE);
        gearShifter = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.MOVE_TO_OUTER_ARMS, Constants.MOVE_TO_INNER_ARM);

        this.driverStation = Robot.secondDS;
        this.leftJoy = Robot.leftJoy;

        firstStageUp = false;
        secondStageUp = false;

        secondStageGearLock.set(DoubleSolenoid.Value.kForward);
        secondStageRelease.set(DoubleSolenoid.Value.kForward);
        firstStageGearLock.set(DoubleSolenoid.Value.kReverse);
        gearShifter.set(Value.kForward);  

        firstStageLeftSwitch = new DigitalInput(Constants.FIRST_STAGE_LIMIT_SWITCH_L);
        firstStageRightSwitch = new DigitalInput(Constants.FIRST_STAGE_LIMIT_SWITCH_R);
        secondStageSwitch = new DigitalInput(Constants.SECOND_STAGE_LIMIT_SWITCH);
    }



    public void teleop() {
        if (driverStation.getRawButton(13)) {
            if (leftJoy.getRawButton(3)) {
                climbingMotor1.set(Constants.CLIMBING_MAX_SPEED);
                climbingMotor2.set(Constants.CLIMBING_MAX_SPEED);
            } else if (leftJoy.getRawButton(4)) {
                climbingMotor1.set(-Constants.CLIMBING_MAX_SPEED);
                climbingMotor2.set(-Constants.CLIMBING_MAX_SPEED);
            } else {
                climbingMotor1.set(0);
                climbingMotor2.set(0);
            }

            if (driverStation.getRawButton(14)) {
                secondStageGearLock.set(Value.kForward);
            }
        } else {
            switch((int) (driverStation.getPOV(1))) {
                case (0):
                    climbingMotor1.set(0);
                    climbingMotor2.set(0);
                    break;            
                case(45):
                    if (!firstStageUp) {
                        climbingMotor1.set(-Constants.CLIMBING_SLOW_SPEED);
                        climbingMotor2.set(-Constants.CLIMBING_SLOW_SPEED);
                        if (!(firstStageLeftSwitch.get() && firstStageRightSwitch.get())) {
                            firstStageUp = true;
                            System.out.println("Pressed");
                        }
                    } else { 
                        climbingMotor1.set(0);
                        climbingMotor2.set(0);
                    }
                    break;
                case(90):
                    // Unlock Gears
                    if (firstStageGearLock.get() != Value.kForward || secondStageGearLock.get() != Value.kReverse) {
                        firstStageGearLock.set(DoubleSolenoid.Value.kForward);
                        secondStageGearLock.set(DoubleSolenoid.Value.kReverse);
                        startTime = System.currentTimeMillis();
                    } else if ((System.currentTimeMillis() - startTime >= 500) && (gearShifter.get() == Value.kForward)) {
                        gearShifter.set(Value.kReverse);
                    }
                    break;
                case(135):
                    // send out third arm 
                    if (!secondStageUp) {
                        climbingMotor1.set(-Constants.CLIMBING_SLOW_SPEED);
                        climbingMotor2.set(-Constants.CLIMBING_SLOW_SPEED);

                        if (!secondStageSwitch.get()) {
                            secondStageUp = true;
                            System.out.println("Second stage bueno");
                        }
                    } else {
                        climbingMotor1.set(0);
                        climbingMotor2.set(0);
                    }
                    break;
                case(180):
                    // disable piston for third arm
                    secondStageRelease.set(Value.kReverse);
                    break;
                case(225):
                    // pull in third arm
                    climbingMotor1.set(Constants.CLIMBING_MAX_SPEED);
                    climbingMotor2.set(Constants.CLIMBING_MAX_SPEED);
                    break;
                case(270):
                    // hold in place
                    climbingMotor1.set(0);
                    climbingMotor2.set(0);
                    break;
                case(315):
                    break;
            }
        }
    }
}
