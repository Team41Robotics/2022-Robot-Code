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
 *      <ul>
 *      <li>secondStageGearLock: fwd - locked; rev - unlocked </li>
 *      <li>firstStageGearLock: fwd - locked; rev - unlocked </li>
 *      <li>secondStageRelease: fwd - locked; rev - unlocked </li>
 *      <li>gearShifter: fwd - first stage; rev - second stage </li>
 *      </ul>
 */
public class Climber {
    public static boolean climbing;
    private static boolean firstStageUp, secondStageUp;
    private static int climbingState;
    private static double motorSpeed;
    private static long startTime;
    private static CANSparkMax climbingMotor1;
    private static CANSparkMax climbingMotor2;
    private static DigitalInput firstStageLeftSwitch, firstStageRightSwitch, secondStageSwitch, secondStageSecondSwitch;
    private static DoubleSolenoid secondStageGearLock, firstStageGearLock, secondStageRelease , gearShifter; // secondStageRelease is second stage piston
    private static Joystick leftJoy;
    private static Joystick driverStation;

    /**
     * Create a new object for controlling the climber on the robot
     */
    public static void initClimber() {
        climbingMotor1 = new CANSparkMax(Constants.CLIMBING_SPARK_F, MotorType.kBrushless);
        climbingMotor2 = new CANSparkMax(Constants.CLIMBING_SPARK_B, MotorType.kBrushless);
        climbingMotor1.setIdleMode(IdleMode.kBrake);
        climbingMotor2.setIdleMode(IdleMode.kBrake);
        
        secondStageGearLock = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.INNER_GEAR_LOCK_OFF, Constants.INNER_GEAR_LOCK_ON);
        firstStageGearLock = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.OUTER_GEAR_LOCK_OFF, Constants.OUTER_GEAR_LOCK_ON);
        secondStageRelease = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.MIDDLE_ARM_LOCK, Constants.MIDDLE_ARM_RELEASE);
        gearShifter = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.MOVE_TO_OUTER_ARMS, Constants.MOVE_TO_INNER_ARM);

        driverStation = Robot.secondDS;
        leftJoy = Robot.leftJoy;

        firstStageUp = false;
        secondStageUp = false;

        secondStageGearLock.set(DoubleSolenoid.Value.kForward);
        secondStageRelease.set(DoubleSolenoid.Value.kForward);
        firstStageGearLock.set(DoubleSolenoid.Value.kReverse);
        gearShifter.set(Value.kForward);  

        firstStageLeftSwitch = new DigitalInput(Constants.FIRST_STAGE_LIMIT_SWITCH_L);
        firstStageRightSwitch = new DigitalInput(Constants.FIRST_STAGE_LIMIT_SWITCH_R);
        secondStageSecondSwitch = new DigitalInput(Constants.FIRST_STAGE_LIMIT_SWTICH_M);
        secondStageSwitch = new DigitalInput(Constants.SECOND_STAGE_LIMIT_SWITCH);
        
        climbing = false;
    }

    /**
     * Reset the climbing process
     */
    public static void reset() {
        secondStageGearLock.set(DoubleSolenoid.Value.kForward);
        secondStageRelease.set(DoubleSolenoid.Value.kForward);
        firstStageGearLock.set(DoubleSolenoid.Value.kReverse);
        gearShifter.set(Value.kForward); 
        firstStageUp = false; 
        secondStageUp = false;
        climbing = false;
    }

    /**
     * Conduct the climbing process using the joysticks and bottom touchscreen, with control being decided by a toggle switch
     */
    public static void teleop() {
        if (driverStation.getRawButton(Controls.SecondDriverStation.MANUAL_CLIMBING_TOGGLE)) {
            if (leftJoy.getRawButton(Controls.LeftJoy.CLIMB_FWD)) {
                motorSpeed = Constants.CLIMBING_MAX_SPEED;
            } else if (leftJoy.getRawButton(Controls.LeftJoy.CLIMB_RV)) {
                motorSpeed = -Constants.CLIMBING_MAX_SPEED;
            } else {
                motorSpeed = 0;
            }

            if (driverStation.getRawButton(Controls.SecondDriverStation.ENABLE_PISTON_BRAKE)) {
                secondStageGearLock.set(Value.kForward);
            }
        } else {
            climbingState = driverStation.getPOV(Controls.SecondDriverStation.CLIMBING_STATE_POV);
            switch(climbingState) {
                case (0):
                    motorSpeed = 0;
                    break;            
                case(45):
                    climbing = true;
                    Hood.setToPosition(0);
                    if (!firstStageUp) {
                        motorSpeed = -Constants.CLIMBING_SLOW_SPEED;
                        if (!firstStageLeftSwitch.get() || !firstStageRightSwitch.get()) {
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
                    } else if ((System.currentTimeMillis() - startTime >= Constants.CLIMBING_PISTON_TIME_DELAY) && (gearShifter.get() == Value.kForward)) {
                        gearShifter.set(Value.kReverse);
                    }
                    break;
                case(135):
                    // send out third arm 
                    if (!secondStageUp) {
                        motorSpeed = -Constants.CLIMBING_SPEED_SECOND_STAGE;

                        if (!secondStageSwitch.get() || !secondStageSecondSwitch.get()) {
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
        }
        climbingMotor1.set(motorSpeed);
        climbingMotor2.set(motorSpeed);
    }

    /**
     * Get the value of the leftmost limit switch
     * @return the value of the switch
     */
    public static boolean getLSwitch() {
        return firstStageLeftSwitch.get();
    }

    /**
     * Get the value of the rightmost switch
     * @return the value of the switch
     */
    public static boolean getRSwitch() {
        return firstStageRightSwitch.get();
    }

    /**
     * Get the value of the second switch for the middle climber
     * @return the value of the switch
     */
    public static boolean getSecondMSwitch() {
        return secondStageSecondSwitch.get();
    }

    /**
     * Get the value of the first switch for the middle climber
     * @return the value of the switch
     */
    public static boolean getMSwitch() {
        return secondStageSwitch.get();
    }

    /**
     * Add all telemetry data for the climber
     * @param table the base telemetry networktable
     */
    public static void telemetry(NetworkTable table) {
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

        NetworkTable secondStageSecondSwitchTable = switchTable.getSubTable("Second Climber Stage Second Limit Switch");
        secondStageSecondSwitchTable.getEntry("name").setString("Second Climber Stage Second Limit Switch");
        secondStageSecondSwitchTable.getEntry("status").setBoolean(secondStageSecondSwitch.get());

        
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
