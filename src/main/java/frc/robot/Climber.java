package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
    private CANSparkMax firstStageMotor1;
    private CANSparkMax firstStageMotor2;
    private Joystick leftJoy;

    public Climber() {
        firstStageMotor1 = new CANSparkMax(Constants.FIRST_STAGE_SPARK_F, MotorType.kBrushless);
        firstStageMotor2 = new CANSparkMax(Constants.FIRST_STAGE_SPARK_B, MotorType.kBrushless);
        firstStageMotor1.setIdleMode(IdleMode.kBrake);
        firstStageMotor2.setIdleMode(IdleMode.kBrake);

        this.leftJoy = Robot.leftJoy;
    }

    public void moveIntake(double speed) {
        firstStageMotor1.set(speed);
        firstStageMotor2.set(speed);
        SmartDashboard.putNumber("Motor 1 set speed", firstStageMotor1.get());
    }

    public void teleop() {
        SmartDashboard.putNumber("Current 1", firstStageMotor1.getOutputCurrent());
        SmartDashboard.putNumber("Current 2", firstStageMotor2.getOutputCurrent());
        if (leftJoy.getRawButton(Constants.FIRST_STAGE_CLIMBING_UP)) {
            this.moveIntake(Constants.CLIMBING_SPEED);
        } else if (leftJoy.getRawButton(Constants.FIRST_STAGE_CLIMBING_DOWN)) {
            this.moveIntake(-Constants.CLIMBING_SPEED);
        } else {
            this.moveIntake(0);
        }
    }
}
