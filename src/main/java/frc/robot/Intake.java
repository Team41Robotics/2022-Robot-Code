package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Intake {
    private DoubleSolenoid intakeSolLeft;
    private DoubleSolenoid intakeSolRight;
    private TalonSRX intakeMotor;
    private Joystick rightJoy;
    private Joystick leftJoy;
    private Boolean intakeOn;
    private Boolean intakeUp;

    /** Initialize all parts of the intake */
    public Intake(){
        intakeSolLeft = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.CTREPCM, Constants.LEFT_SOL_FWD, Constants.LEFT_SOL_RV);
        intakeSolRight = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.CTREPCM, Constants.RIGHT_SOL_FWD, Constants.RIGHT_SOL_RV);
        intakeMotor = new TalonSRX(Constants.INTAKE_MOTOR);
        rightJoy = Robot.rightJoy;
        leftJoy = Robot.leftJoy;
        intakeOn = false;
        intakeUp = false;
    }

    /** At beginning of auton, move the intake down and start the motor */
    public void autonInit(){
        intakeSolLeft.set(DoubleSolenoid.Value.kForward);
        intakeSolRight.set(DoubleSolenoid.Value.kForward);
        intakeUp = true;
        intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_FULL_SPEED);
    }

    /** Set the intake motor to a specified speed between -1 and 1 */
    public void setIntakeMotor(double speed){
        intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    /** In teleop, set buttons to raise the intake and toggle the motor */
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

