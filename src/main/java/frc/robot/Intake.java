package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

/** Controls the intake on the robot */
public class Intake {
    private boolean intakeOn;
    private boolean intakeUp;
    private Joystick leftJoy;
    private Joystick rightJoy;
    private TalonSRX intakeMotor;
    private DoubleSolenoid intakeSolLeft;
    private DoubleSolenoid intakeSolRight;

    /** Initialize all parts of the intake */
    public Intake(){
        intakeOn = false;
        intakeUp = false;
        leftJoy = Robot.leftJoy;
        rightJoy = Robot.rightJoy;
        intakeMotor = new TalonSRX(Constants.INTAKE_MOTOR);
        intakeSolLeft = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.LEFT_SOL_FWD, Constants.LEFT_SOL_RV);
        intakeSolRight = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.RIGHT_SOL_FWD, Constants.RIGHT_SOL_RV);
    }

    /** At beginning of auton, move the intake down and start the motor */
    public void autonInit(){
        intakeSolLeft.set(DoubleSolenoid.Value.kForward);
        intakeSolRight.set(DoubleSolenoid.Value.kForward);
        intakeUp = true;
        intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_FULL_SPEED);
    }

    /**
     * Set the intake motor to a specified speed
     * @param speed desired speed of the intake motor [-1, 1]
     */
    public void setIntakeMotor(double speed){
        intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    /** In teleop, use joystick triggers to raise/lower the intake and toggle the motor */
    public void teleop(){
        if (leftJoy.getRawButtonPressed(1)) {
            intakeUp = !intakeUp;
            intakeSolLeft.set(intakeUp ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
            intakeSolRight.set(intakeUp ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
          }
          if (rightJoy.getRawButtonPressed(1)) {
            intakeOn = !intakeOn;
            intakeMotor.set(TalonSRXControlMode.PercentOutput, intakeOn ? Constants.INTAKE_FULL_SPEED : 0);
          }
    }

}   

