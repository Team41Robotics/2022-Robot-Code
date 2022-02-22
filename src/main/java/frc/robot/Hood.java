package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class Hood {
    CANSparkMax hoodMotor;
    DigitalInput topSwitch;
    DigitalInput bottomSwitch;
    DigitalInput climber;

    public Hood() {
        hoodMotor = new CANSparkMax(Constants.HOOD_SPARK, MotorType.kBrushless);
        topSwitch = new DigitalInput(Constants.HOOD_TOP_LIMIT_SWITCH);
        bottomSwitch = new DigitalInput(Constants.HOOD_BOTTOM_LIMIT_SWITCH);
        climber = new DigitalInput(2);
        hoodMotor.setIdleMode(IdleMode.kBrake);
    }

    public void teleop() {
        System.out.print(topSwitch.get());
        System.out.print(bottomSwitch.get());
        System.out.println(climber.get());
    }
}
