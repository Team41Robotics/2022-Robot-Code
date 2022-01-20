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

    public Intake(){
        intakeSolLeft = new DoubleSolenoid(15, PneumaticsModuleType.CTREPCM, 3, 2);
        intakeSolRight = new DoubleSolenoid(15, PneumaticsModuleType.CTREPCM, 5, 4);
        intakeMotor = new TalonSRX(14);
        rightJoy = Robot.rightJoy;
        leftJoy = Robot.leftJoy;
        intakeOn = false;
    }

    public void autonInit(){
        intakeSolLeft.set(DoubleSolenoid.Value.kForward);
        intakeSolRight.set(DoubleSolenoid.Value.kForward);
        intakeMotor.set(ControlMode.PercentOutput, 0.6);
    }

    public void setIntakeMotor(double speed){
        intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void teleop(){
        if (leftJoy.getRawButtonPressed(1)) {
            intakeSolLeft.set(DoubleSolenoid.Value.kReverse);
            intakeSolRight.set(DoubleSolenoid.Value.kReverse);
          }
          if (rightJoy.getRawButtonPressed(1)) {
            intakeOn = !intakeOn;
            intakeMotor.set(TalonSRXControlMode.PercentOutput, intakeOn ? 0.6 : 0);
          }
    }

}   

