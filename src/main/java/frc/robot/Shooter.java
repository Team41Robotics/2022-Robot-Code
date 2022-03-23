package frc.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter {
    public static boolean reverseOn = false;
    public static boolean inUse = Intake.inUse;
    public boolean testOn = false;
    public PID leftFalconPID, rightFalconPID;
    public double speed = 0;
    private CANSparkMax feeder, elevator;
    private Joystick rightDS, rightJoy;
    private TalonFX leftFalcon, rightFalcon;
    
    public Shooter() {
        leftFalcon = new TalonFX(Constants.SHOOTER_TALON_2);
        rightFalcon = new TalonFX(Constants.SHOOTER_TALON_1);
        rightFalcon.setInverted(true);
        leftFalcon.setInverted(false);
        // maybe add some D
        leftFalconPID = new PID(leftFalcon, Constants.SHOOTER_kP, Constants.SHOOTER_kI, Constants.SHOOTER_kD, Constants.SHOOTER_kFF, Constants.SHOOTER_RAMP_TIME);
        rightFalconPID = new PID(rightFalcon, Constants.SHOOTER_kP, Constants.SHOOTER_kI, Constants.SHOOTER_kD, Constants.SHOOTER_kFF, Constants.SHOOTER_RAMP_TIME);
        feeder = new CANSparkMax(Constants.FEEDER_MOTOR, MotorType.kBrushless);
        elevator = new CANSparkMax(Constants.ELEVATOR_MOTOR, MotorType.kBrushless);
        speed = 0;
        rightJoy = Robot.rightJoy;
        reverseOn = false;
        rightDS = Robot.secondDS;
    }

    public void teleop() {
        if (rightJoy.getRawButton(Controls.RightJoy.INTAKE_REVERSE)) {
            reverseOn = true;
            elevator.set(reverseOn ? -Constants.ELEVATOR_FULL_SPEED : 0);
            feeder.set(reverseOn ? -Constants.FEEDER_FULL_SPEED : 0);
        } else {
            reverseOn = false;
            if (rightDS.getRawButton(Controls.SecondDriverStation.FEED_BALL_TO_SHOOTER) && reverseOn == false) {
                feeder.set(Constants.FEEDER_FULL_SPEED);
            } else {
                feeder.set(reverseOn ? -Constants.FEEDER_FULL_SPEED : 0);
                
            }
        }
        SmartDashboard.putNumber("Shooter Error", leftFalconPID.getError());
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

    public double getErr() {
        return leftFalconPID.getError();
    }

    public void runElevator(double speed) {
        elevator.set(speed);
    }

    public void telemetry(NetworkTable table) {
        NetworkTable motorTable = table.getSubTable("motors");

        leftFalconPID.telemetry(motorTable, "Left Shooter Motor");
        rightFalconPID.telemetry(motorTable, "Right Shooter Motor");
        
        NetworkTable feederMotorTable = motorTable.getSubTable("Feeder Motor");
        feederMotorTable.getEntry("name").setString("Feeder Motor");
        feederMotorTable.getEntry("loop_error").setDouble(-1);
        feederMotorTable.getEntry("p").setDouble(-1);
        feederMotorTable.getEntry("i").setDouble(-1);
        feederMotorTable.getEntry("d").setDouble(-1);
        feederMotorTable.getEntry("requested_input_speed").setDouble(-1);
        feederMotorTable.getEntry("actual_input_speed").setDouble(-1);
        feederMotorTable.getEntry("raw_input_speed").setDouble(feeder.get());
        feederMotorTable.getEntry("output_speed").setDouble(feeder.getEncoder().getVelocity());
        feederMotorTable.getEntry("position").setDouble(feeder.getEncoder().getPosition());
        feederMotorTable.getEntry("current").setDouble(feeder.getOutputCurrent());
        
        NetworkTable elevatorMotorTable = motorTable.getSubTable("Elevator Motor");
        elevatorMotorTable.getEntry("name").setString("Elevator Motor");
        elevatorMotorTable.getEntry("loop_error").setDouble(-1);
        elevatorMotorTable.getEntry("p").setDouble(-1);
        elevatorMotorTable.getEntry("i").setDouble(-1);
        elevatorMotorTable.getEntry("d").setDouble(-1);
        elevatorMotorTable.getEntry("requested_input_speed").setDouble(-1);
        elevatorMotorTable.getEntry("actual_input_speed").setDouble(-1);
        elevatorMotorTable.getEntry("raw_input_speed").setDouble(elevator.get());
        elevatorMotorTable.getEntry("output_speed").setDouble(elevator.getEncoder().getVelocity());
        elevatorMotorTable.getEntry("position").setDouble(elevator.getEncoder().getPosition());
        elevatorMotorTable.getEntry("current").setDouble(elevator.getOutputCurrent());
    }
}
