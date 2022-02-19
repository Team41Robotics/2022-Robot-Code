package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter {
    TalonFX leftFalcon, rightFalcon;
    CANSparkMax intake, conveyor, feeder, elevator;
    Joystick rightDS;
    PID leftFalconPID, rightFalconPID;
    
    public Shooter() {
        leftFalcon = new TalonFX(Constants.SHOOTER_TALON_2);
        rightFalcon = new TalonFX(Constants.SHOOTER_TALON_1);
        rightFalcon.setInverted(true);
        leftFalcon.setInverted(false);
        // maybe add some D
        leftFalconPID = new PID(leftFalcon, 0.5, 0.03, 0.0005, 1.1, 1);
        rightFalconPID = new PID(rightFalcon, 0.5, 0.03, 0.0005, 1.1, 1);

        intake = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
        conveyor = new CANSparkMax(Constants.CONVEYOR_MOTOR, MotorType.kBrushless);
        feeder = new CANSparkMax(Constants.FEEDER_MOTOR, MotorType.kBrushless);
        elevator = new CANSparkMax(Constants.ELEVATOR_MOTOR, MotorType.kBrushless);
        conveyor.setInverted(true);

        rightDS = Robot.secondDS;
    }

    public void teleop() {
        SmartDashboard.putNumber("Shooter 1 Supply Current", leftFalcon.getSupplyCurrent());
        SmartDashboard.putNumber("Shooter 1 Stator Current", leftFalcon.getStatorCurrent());
        SmartDashboard.putNumber("Shooter 2 Stator Current", rightFalcon.getStatorCurrent());
        SmartDashboard.putNumber("Shooter 2 Supply Current", rightFalcon.getSupplyCurrent());
        SmartDashboard.putNumber("Shooter 1 Speed", leftFalcon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooter 2 Speed", rightFalcon.getSelectedSensorVelocity());
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
}
