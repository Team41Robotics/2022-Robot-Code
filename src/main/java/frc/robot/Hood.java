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
    public double angle;

    public Hood() {
        // Bottom switch needs to be inverted
        hoodMotor = new CANSparkMax(Constants.HOOD_SPARK, MotorType.kBrushless);
        topSwitch = new DigitalInput(Constants.HOOD_TOP_LIMIT_SWITCH);
        bottomSwitch = new DigitalInput(Constants.HOOD_BOTTOM_LIMIT_SWITCH);
        hoodMotor.setIdleMode(IdleMode.kBrake);
        enc = hoodMotor.getEncoder();
        enc.setPosition(0);
        station = Robot.secondDS;
        angle = 0;
    }

    public void teleop() {
        double pos = enc.getPosition();
        if (!topSwitch.get() && station.getRawButton(7) && pos <= 50) {
            hoodMotor.set(Constants.HOOD_SPEED);
        } else if (bottomSwitch.get() && station.getRawButton(8)) {
            hoodMotor.set(-Constants.HOOD_SPEED/2);
        } else {
            hoodMotor.set(0);
        }
    }

    private void setToPosition(double pos, double angle) {
        if (!topSwitch.get() && (pos - angle) < -1) {
            hoodMotor.set(Constants.HOOD_SPEED/2);
        } else if (bottomSwitch.get() && (pos-angle) > 1) {
            hoodMotor.set(-Constants.HOOD_SPEED/4);
        } else {
            hoodMotor.set(0);
        }
    }

    // Adjust angle based on switch
    public void test() {
        double pos = enc.getPosition();
        if (!topSwitch.get() && station.getRawButtonPressed(9) && pos <= 50) {
            angle += (angle < 50) ? 1 : 0;
        } else if (bottomSwitch.get() && station.getRawButtonPressed(10)) {
            angle -= (angle > 0) ? 1 : 0;
        }

        setToPosition(pos, angle);
    }

    public void home() {
        while (bottomSwitch.get()) {
            hoodMotor.set(-Constants.HOOD_SPEED/4);
        }
        enc.setPosition(0);
    }
}
