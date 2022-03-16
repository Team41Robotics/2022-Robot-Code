package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

/** Pistons:
 *      secondStageGearLock: fwd - locked; rev - unlocked
 *      firstStageGearLock: fwd - locked; rev - unlocked
 *      secondStageRelease: fwd - locked; rev - unlocked
 *      gearShifter: fwd - first stage; rev - second stage
 */


public class Climber {
    public boolean climbing;
    private boolean firstStageUp, secondStageUp;
    private int climbingState;
    private double motorSpeed;
    private long startTime;
    private CANSparkMax climbingMotor1;
    private CANSparkMax climbingMotor2;
    private DigitalInput firstStageLeftSwitch, firstStageRightSwitch, secondStageSwitch, firstStageMidSwitch;
    private DoubleSolenoid secondStageGearLock, firstStageGearLock, secondStageRelease , gearShifter; // secondStageRelease is second stage piston
    private Hood hood;
    private Joystick leftJoy;
    private Joystick driverStation;

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
        this.hood = Robot.hood;

        firstStageUp = false;
        secondStageUp = false;

        secondStageGearLock.set(DoubleSolenoid.Value.kForward);
        secondStageRelease.set(DoubleSolenoid.Value.kForward);
        firstStageGearLock.set(DoubleSolenoid.Value.kReverse);
        gearShifter.set(Value.kForward);  

        firstStageLeftSwitch = new DigitalInput(Constants.FIRST_STAGE_LIMIT_SWITCH_L);
        firstStageRightSwitch = new DigitalInput(Constants.FIRST_STAGE_LIMIT_SWITCH_R);
        firstStageMidSwitch = new DigitalInput(Constants.FIRST_STAGE_LIMIT_SWTICH_M);
        secondStageSwitch = new DigitalInput(Constants.SECOND_STAGE_LIMIT_SWITCH);
        
        climbing = false;
    }

    public void reset() {
        secondStageGearLock.set(DoubleSolenoid.Value.kForward);
        secondStageRelease.set(DoubleSolenoid.Value.kForward);
        firstStageGearLock.set(DoubleSolenoid.Value.kReverse);
        gearShifter.set(Value.kForward); 
        firstStageUp = false; 
        secondStageUp = false;
        climbing = false;
    }

    public void teleop() {
        if (driverStation.getRawButton(13)) {
            if (leftJoy.getRawButton(3)) {
                motorSpeed = Constants.CLIMBING_MAX_SPEED;
            } else if (leftJoy.getRawButton(4)) {
                motorSpeed = -Constants.CLIMBING_MAX_SPEED;
            } else {
                motorSpeed = 0;
            }

            if (driverStation.getRawButton(14)) {
                secondStageGearLock.set(Value.kForward);
            }
        } else {
            climbingState = driverStation.getPOV(1);
            switch(climbingState) {
                case (0):
                    motorSpeed = 0;
                    break;            
                case(45):
                    climbing = true;
                    hood.setToPosition(0);
                    if (!firstStageUp) {
                        motorSpeed = -Constants.CLIMBING_SLOW_SPEED;
                        if (!(firstStageLeftSwitch.get() || firstStageRightSwitch.get())) {
                            firstStageUp = true;
                            System.out.println("Pressed");
                        }
                    } else { 
                        motorSpeed = 0;
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
                        motorSpeed = -Constants.CLIMBING_SLOW_SPEED;

                        if (!secondStageSwitch.get()) {
                            secondStageUp = true;
                            System.out.println("Second stage bueno");
                        }
                    } else {
                        motorSpeed = 0;
                    }
                    break;
                case(180):
                    // disable piston for third arm
                    secondStageRelease.set(Value.kReverse);
                    break;
                case(225):
                    // hold in place
                    motorSpeed = 0;
                    break;
                case(270):
                    break;
                case(315):
                    break;
            }
            climbingMotor1.set(motorSpeed);
            climbingMotor2.set(motorSpeed);
        }
    }

    public boolean getLSwitch() {
        return firstStageLeftSwitch.get();
    }

    public boolean getRSwitch() {
        return firstStageRightSwitch.get();
    }

    public boolean getMSwitch() {
        return firstStageMidSwitch.get();
    }

    public void telemetry(NetworkTable table) {
        NetworkTable motorTable = table.getSubTable("motors");
        
        NetworkTable climberMotor1Table = motorTable.getSubTable("Climber Motor 1");
        climberMotor1Table.getEntry("name").setString("Climber Motor 1");
        climberMotor1Table.getEntry("loop_error").setDouble(-1);
        climberMotor1Table.getEntry("p").setDouble(-1);
        climberMotor1Table.getEntry("i").setDouble(-1);
        climberMotor1Table.getEntry("d").setDouble(-1);
        climberMotor1Table.getEntry("requested_input_speed").setDouble(-1);
        climberMotor1Table.getEntry("actual_input_speed").setDouble(-1);
        climberMotor1Table.getEntry("raw_input_speed").setDouble(climbingMotor1.get());
        climberMotor1Table.getEntry("output_speed").setDouble(climbingMotor1.getEncoder().getVelocity());
        climberMotor1Table.getEntry("position").setDouble(climbingMotor1.getEncoder().getPosition());
        climberMotor1Table.getEntry("current").setDouble(climbingMotor1.getOutputCurrent());
       
        NetworkTable climberMotor2Table = motorTable.getSubTable("Climber Motor 2");
        climberMotor2Table.getEntry("name").setString("Climber Motor 2");
        climberMotor2Table.getEntry("loop_error").setDouble(-1);
        climberMotor2Table.getEntry("p").setDouble(-1);
        climberMotor2Table.getEntry("i").setDouble(-1);
        climberMotor2Table.getEntry("d").setDouble(-1);
        climberMotor2Table.getEntry("requested_input_speed").setDouble(-1);
        climberMotor2Table.getEntry("actual_input_speed").setDouble(-1);
        climberMotor2Table.getEntry("raw_input_speed").setDouble(climbingMotor2.get());
        climberMotor2Table.getEntry("output_speed").setDouble(climbingMotor2.getEncoder().getVelocity());
        climberMotor2Table.getEntry("position").setDouble(climbingMotor2.getEncoder().getPosition());
        climberMotor2Table.getEntry("current").setDouble(climbingMotor2.getOutputCurrent());

        
        NetworkTable switchTable = table.getSubTable("limit_switches");
        
        NetworkTable firstStageLeftSwitchTable = switchTable.getSubTable("First Climber Stage Left Limit Switch");
        firstStageLeftSwitchTable.getEntry("name").setString("First Climber Stage Left Limit Switch");
        firstStageLeftSwitchTable.getEntry("status").setBoolean(firstStageLeftSwitch.get());

        NetworkTable firstStageRightSwitchTable = switchTable.getSubTable("First Climber Stage Right Limit Switch");
        firstStageRightSwitchTable.getEntry("name").setString("First Climber Stage Right Limit Switch");
        firstStageRightSwitchTable.getEntry("status").setBoolean(firstStageRightSwitch.get());
                
        NetworkTable secondStageSwitchTable = switchTable.getSubTable("Second Climber Stage Limit Switch");
        secondStageSwitchTable.getEntry("name").setString("Second Climber Stage Limit Switch");
        secondStageSwitchTable.getEntry("status").setBoolean(secondStageSwitch.get());

        
        NetworkTable solenoidTable = table.getSubTable("solenoids");

        NetworkTable secondStageGearLockTable = solenoidTable.getSubTable("Second Stage Friction Brake");
        secondStageGearLockTable.getEntry("name").setString("Second Stage Friction Brake");
        secondStageGearLockTable.getEntry("status").setDouble(secondStageGearLock.get() == Value.kForward ? 1 : (secondStageGearLock.get() == Value.kReverse ? -1 : 0));
        
        NetworkTable firstStageGearLockTable = solenoidTable.getSubTable("Gear Jammer");
        firstStageGearLockTable.getEntry("name").setString("Gear Jammer");
        firstStageGearLockTable.getEntry("status").setDouble(firstStageGearLock.get() == Value.kForward ? 1 : (firstStageGearLock.get() == Value.kReverse ? -1 : 0));

        NetworkTable secondStageReleaseTable = solenoidTable.getSubTable("Second Stage Release");
        secondStageReleaseTable.getEntry("name").setString("Second Stage Release");
        secondStageReleaseTable.getEntry("status").setDouble(secondStageRelease.get() == Value.kForward ? 1 : (secondStageRelease.get() == Value.kReverse ? -1 : 0));

        NetworkTable gearShifterTable = solenoidTable.getSubTable("Gear Shifter");
        gearShifterTable.getEntry("name").setString("Gear Shifter");
        gearShifterTable.getEntry("status").setDouble(gearShifter.get() == Value.kForward ? 1 : (gearShifter.get() == Value.kReverse ? -1 : 0));


        NetworkTableEntry climbingStateEntry = table.getEntry("state_climber");
        switch (climbingState) {
            case 0:
                climbingStateEntry.setDouble(-1);
                break;
            case 45:
                climbingStateEntry.setDouble(0);
                break;
            case 90:
                climbingStateEntry.setDouble(1);
                break;
            case 135:
                climbingStateEntry.setDouble(2);
                break;
            case 180:
                climbingStateEntry.setDouble(3);
                break;
            case 225:
                climbingStateEntry.setDouble(4);
        }
    }
}
