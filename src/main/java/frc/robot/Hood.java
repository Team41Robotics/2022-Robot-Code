package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

public class Hood {
    CANSparkMax hoodMotor;
    DigitalInput topSwitch, bottomSwitch;
    Joystick station;
    RelativeEncoder enc;

    public Hood() {
        // Bottom switch needs to be inverted
        hoodMotor = new CANSparkMax(Constants.HOOD_SPARK, MotorType.kBrushless);
        topSwitch = new DigitalInput(Constants.HOOD_TOP_LIMIT_SWITCH);
        bottomSwitch = new DigitalInput(Constants.HOOD_BOTTOM_LIMIT_SWITCH);
        hoodMotor.setIdleMode(IdleMode.kBrake);
        enc = hoodMotor.getEncoder();
        enc.setPosition(0);
        station = Robot.secondDS;
    }

    public void teleop() {
        // System.out.print(topSwitch.get());
        // System.out.println(!bottomSwitch.get());
        if (!topSwitch.get() && station.getRawButton(9)) {
            hoodMotor.set(Constants.HOOD_SPEED);
        } else if (bottomSwitch.get() && station.getRawButton(10)) {
            hoodMotor.set(-Constants.HOOD_SPEED/2);
        } else {
            hoodMotor.set(0);
        }
        // System.out.println(enc.getPosition());
    }
}
