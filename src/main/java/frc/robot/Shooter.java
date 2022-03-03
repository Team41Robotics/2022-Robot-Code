package frc.robot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter {
    TalonFX leftFalcon, rightFalcon;
    CANSparkMax feeder, elevator;
    Joystick rightDS, rightJoy;
    public PID leftFalconPID, rightFalconPID;
    public double speed = 0;
    public boolean testOn = false;
    public static boolean reverseOn = false;
    Intake intake;
    
    public Shooter() {
        leftFalcon = new TalonFX(Constants.SHOOTER_TALON_2);
        rightFalcon = new TalonFX(Constants.SHOOTER_TALON_1);
        rightFalcon.setInverted(true);
        leftFalcon.setInverted(false);
        // maybe add some D
        leftFalconPID = new PID(leftFalcon, 0.5, 0.03, 0.0005, 1.1, 1);
        rightFalconPID = new PID(rightFalcon, 0.5, 0.03, 0.0005, 1.1, 1);
        feeder = new CANSparkMax(Constants.FEEDER_MOTOR, MotorType.kBrushless);
        elevator = new CANSparkMax(Constants.ELEVATOR_MOTOR, MotorType.kBrushless);
        speed = 0;
        rightJoy = Robot.rightJoy;
        intake = Robot.intake;
        reverseOn = false;
        rightDS = Robot.secondDS;
    }

    public void teleop() {
        if (rightJoy.getRawButton(3)) {
            reverseOn = true;
        } else {
            reverseOn = false;
            if (rightDS.getRawButton(1) && reverseOn == false) {
                feeder.set(Constants.FEEDER_FULL_SPEED);
            }
            elevator.set(reverseOn ? -Constants.ELEVATOR_FULL_SPEED : 0);
            feeder.set(reverseOn ? -Constants.FEEDER_FULL_SPEED : 0);
        }
    }

    public void test() {
        if (rightDS.getRawButtonPressed(11)) {
            speed += (speed < 0.89) ? 0.025 : 0;
        } else if (rightDS.getRawButtonPressed(12)) {
            speed -= (speed > 0.01) ? 0.025 : 0;
        }

        setSpeed(speed);

        if (rightJoy.getRawButtonPressed(1)) {
            elevator.set(testOn ? Constants.ELEVATOR_FULL_SPEED : 0);
            
            testOn = !testOn;
        }
        if (rightDS.getRawButton(1)) {
            feeder.set(Constants.FEEDER_FULL_SPEED);
        } else {
            feeder.set(0);
        }
    }

    public void setSpeed(double speed) {
        leftFalconPID.run(speed);
        rightFalconPID.run(speed);
        if (speed != 0) {
            elevator.set(Constants.ELEVATOR_FULL_SPEED);
        } else {
            elevator.set(0);
        }
    }

    public boolean isReady() {
        return leftFalconPID.isReady();
    }

    public void runFeeder(boolean on) {
        feeder.set(on ? Constants.FEEDER_FULL_SPEED : 0);
    }
}
