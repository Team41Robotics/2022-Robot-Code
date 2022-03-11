
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.INTAKE_MODE;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Controls the intake on the robot */
public class Intake {
    private boolean intakeOn;
    private boolean intakeUp;
    private Joystick leftJoy, rightJoy, secondDS;
    private CANSparkMax intakeMotor, conveyor;
    private DoubleSolenoid intakeSolLeft;
    private DoubleSolenoid intakeSolRight;
    public static boolean inUse;

    /** Initialize all parts of the intake */
    public Intake(){
        inUse = false;
        intakeOn = false;
        intakeUp = false;
        leftJoy = Robot.leftJoy;
        rightJoy = Robot.rightJoy;
        secondDS = Robot.secondDS;
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
        conveyor = new CANSparkMax(Constants.CONVEYOR_MOTOR, MotorType.kBrushless);
        conveyor.setInverted(true);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeSolLeft = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.LEFT_SOL_FWD, Constants.LEFT_SOL_RV);
        intakeSolRight = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.RIGHT_SOL_FWD, Constants.RIGHT_SOL_RV);
    }

    /** At beginning of auton, move the intake down and start the motor */
    public void autonInit(){
        intakeSolLeft.set(DoubleSolenoid.Value.kForward);
        intakeSolRight.set(DoubleSolenoid.Value.kForward);
        intakeUp = true;
        intakeMotor.set(Constants.INTAKE_FULL_SPEED);
        conveyor.set(Constants.CONVEYOR_FULL_SPEED);
    }

    /**
     * Set the intake motor to a specified speed
     * @param speed desired speed of the intake motor [-1, 1]
     */
    public void setIntakeMotor(double speed){
        intakeMotor.set(speed);
    }

    /** In teleop, use joystick triggers to raise/lower the intake and toggle the motor */
    public void teleop(){
        if (leftJoy.getRawButtonPressed(1)) {
          intakeUp = !intakeUp;
          intakeSolLeft.set(intakeUp ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
          intakeSolRight.set(intakeUp ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
        }
        if (Shooter.reverseOn) {
          conveyor.set(-Constants.CONVEYOR_FULL_SPEED);
          intakeMotor.set(-Constants.INTAKE_FULL_SPEED);
        } else if (rightJoy.getRawButtonPressed(1)) {
          intakeOn = !intakeOn;
          run(intakeOn ? INTAKE_MODE.FORWARD : INTAKE_MODE.OFF);
        } else if (secondDS.getRawButton(1)) {
          conveyor.set(Constants.CONVEYOR_FULL_SPEED);
        } else if (!intakeOn) {
          conveyor.set(0);
          intakeMotor.set(0);
        }

        SmartDashboard.putBoolean("Intake On", intakeOn);
    }

    public void test() {
        if (leftJoy.getRawButtonPressed(1)) {
            intakeUp = !intakeUp;
            intakeSolLeft.set(intakeUp ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
            intakeSolRight.set(intakeUp ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
          }
          if (rightJoy.getRawButtonPressed(1)) {
            intakeOn = !intakeOn;
            run(intakeOn ? INTAKE_MODE.FORWARD : INTAKE_MODE.OFF);
          }
    }

    // true - forward; false - reverse
    public void run(Constants.INTAKE_MODE dir) {
      switch (dir) {
        case FORWARD:
          intakeMotor.set(Constants.INTAKE_FULL_SPEED);
          conveyor.set(Constants.CONVEYOR_FULL_SPEED);
          break;
        case REVERSE:
          intakeMotor.set(-Constants.INTAKE_FULL_SPEED);
          conveyor.set(-Constants.CONVEYOR_FULL_SPEED);
        case OFF:
          intakeMotor.set(0);
          conveyor.set(0);
      }
    }

    public void setIntakeOn(boolean val) {
      intakeOn = val;
    }

    public void runConveyor(boolean on) {
      conveyor.set(on ? Constants.CONVEYOR_FULL_SPEED : 0);
    }

    public void stop() {
      conveyor.set(0);
      intakeMotor.set(0);
    }
}   

