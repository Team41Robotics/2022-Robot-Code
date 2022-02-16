package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;

public class Climber {
    private CANSparkMax firstStageMotor;
    private Joystick leftJoy;

    public Climber() {
        firstStageMotor = new CANSparkMax(Constants.FIRST_STAGE_SPARK, MotorType.kBrushless);
        firstStageMotor.setIdleMode(IdleMode.kBrake);

        this.leftJoy = Robot.leftJoy;
    }

    
    public void teleop() {
        if (leftJoy.getRawButton(Constants.FIRST_STAGE_CLIMBING_BUTTON)) {
            this.firstStageMotor.set(Constants.CLIMBING_SPEED);
        } else {
            this.firstStageMotor.set(0);
        }
    }
}
