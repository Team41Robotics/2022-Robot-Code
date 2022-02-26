package frc.robot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter {
    TalonFX leftFalcon, rightFalcon;
    CANSparkMax intake, conveyor, feeder, elevator;
    Joystick rightDS, rightJoy;
    PID leftFalconPID, rightFalconPID;
    public double speed = 0;
    boolean testOn = false;
    
    public Shooter() {
        leftFalcon = new TalonFX(Constants.SHOOTER_TALON_2);
        rightFalcon = new TalonFX(Constants.SHOOTER_TALON_1);
        rightFalcon.setInverted(true);
        leftFalcon.setInverted(false);
        // maybe add some D
        leftFalconPID = new PID(leftFalcon, 0.5, 0.03, 0.0005, 1.1, 1);
        rightFalconPID = new PID(rightFalcon, 0.5, 0.03, 0.0005, 1.1, 1);
        conveyor = new CANSparkMax(Constants.CONVEYOR_MOTOR, MotorType.kBrushless);
        feeder = new CANSparkMax(Constants.FEEDER_MOTOR, MotorType.kBrushless);
        elevator = new CANSparkMax(Constants.ELEVATOR_MOTOR, MotorType.kBrushless);
        conveyor.setInverted(true);
        speed = 0;
        rightJoy = Robot.rightJoy;

        rightDS = Robot.secondDS;
    }

    public void teleop() {
        /**
         * 1st - conveyor
         * 2nd - elevator and shooter
         * 3rd - feeder wheel
         * 4th - everything
         */
        if (rightDS.getRawButton(4) || rightDS.getRawButton(1)) {
            conveyor.set(Constants.CONVEYOR_FULL_SPEED);
        } else {
            conveyor.set(0);
        }
        if (rightDS.getRawButton(4) || rightDS.getRawButton(2)) {
            elevator.set(Constants.ELEVATOR_FULL_SPEED);
            leftFalconPID.run(Constants.SHOOTER_SPEED);
            rightFalconPID.run(Constants.SHOOTER_SPEED);
        } else {
            elevator.set(0);
            leftFalconPID.run(0);
            rightFalconPID.run(0);
        }
        if (rightDS.getRawButton(4) || rightDS.getRawButton(3)) {
            feeder.set(Constants.FEEDER_FULL_SPEED);
        } else {
            feeder.set(0);
        }
    }

    public void test() {
        if (rightDS.getRawButtonPressed(11)) {
            speed += (speed < 0.89) ? 0.025 : 0;
        } else if (rightDS.getRawButtonPressed(12)) {
            speed -= (speed > 0.01) ? 0.025 : 0;
        }

        leftFalconPID.run(speed);
        rightFalconPID.run(speed);


        if (rightJoy.getRawButtonPressed(1)) {
            conveyor.set(testOn ? Constants.CONVEYOR_FULL_SPEED : 0);
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
    }
}
