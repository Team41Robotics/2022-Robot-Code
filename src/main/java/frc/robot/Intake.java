
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.INTAKE_MODE;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Controls the intake on the robot */
public class Intake {
    public static boolean inUse;
    private boolean intakeOn;
    private boolean intakeUp;
    private CANSparkMax intakeMotor, conveyor;
    private DoubleSolenoid intakeSolLeft;
    private DoubleSolenoid intakeSolRight;
    private Joystick leftJoy, rightJoy, secondDS;
    private ShuffleboardTab robotTab;
    private NetworkTableEntry intakeStatus = robotTab.add("Intake On", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    /** Initialize all parts of the intake */
    public Intake(){
        inUse = false;
        intakeOn = false;
        intakeUp = false;
        leftJoy = Robot.leftJoy;
        rightJoy = Robot.rightJoy;
        secondDS = Robot.secondDS;
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
        conveyor = new CANSparkMax(Constants.CONVEYOR_MOTOR, MotorType.kBrushless);
        conveyor.setInverted(true);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeSolLeft = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.LEFT_SOL_FWD, Constants.LEFT_SOL_RV);
        intakeSolRight = new DoubleSolenoid(Constants.PCM_PORT, PneumaticsModuleType.REVPH, Constants.RIGHT_SOL_FWD, Constants.RIGHT_SOL_RV);
        robotTab = Shuffleboard.getTab("Robot");
    }

    /** At beginning of auton, move the intake down and start the motor */
    public void autonInit(){
        intakeSolLeft.set(DoubleSolenoid.Value.kForward);
        intakeSolRight.set(DoubleSolenoid.Value.kForward);
        intakeUp = true;
        intakeMotor.set(Constants.INTAKE_FULL_SPEED);
        conveyor.set(Constants.CONVEYOR_FULL_SPEED);
    }

    /** In teleop, use joystick triggers to raise/lower the intake and toggle the motor */
    public void teleop(){
        if (leftJoy.getRawButtonPressed(Controls.LeftJoy.INTAKE_PISTON_TOGGLE)) {
          // intakeUp = !intakeUp;
          // intakeSolLeft.set(intakeUp ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
          // intakeSolRight.set(intakeUp ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
        }
        if (Shooter.reverseOn) {
          conveyor.set(-Constants.CONVEYOR_FULL_SPEED);
          intakeMotor.set(-Constants.INTAKE_FULL_SPEED);
        } else if (rightJoy.getRawButtonPressed(Controls.RightJoy.INTAKE_TOGGLE)) {
          intakeUp = !intakeUp;
          intakeSolLeft.set(intakeUp ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
          intakeSolRight.set(intakeUp ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
          intakeOn = !intakeOn;
          run(intakeOn ? INTAKE_MODE.FORWARD : INTAKE_MODE.OFF);
        } else if (secondDS.getRawButton(Controls.SecondDriverStation.FEED_BALL_TO_SHOOTER)) {
          conveyor.set(Constants.CONVEYOR_FULL_SPEED);
        } else if (!intakeOn) {
          conveyor.set(0);
          intakeMotor.set(0);
        }

        intakeStatus.setBoolean(intakeOn);
    }

    public void test() {
        if (leftJoy.getRawButtonPressed(Controls.LeftJoy.INTAKE_PISTON_TOGGLE)) {
            intakeUp = !intakeUp;
            intakeSolLeft.set(intakeUp ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
            intakeSolRight.set(intakeUp ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
          }
          if (rightJoy.getRawButtonPressed(Controls.RightJoy.INTAKE_TOGGLE)) {
            intakeOn = !intakeOn;
            run(intakeOn ? INTAKE_MODE.FORWARD : INTAKE_MODE.OFF);
          }
    }

    // true - forward; false - reverse
    public void run(Constants.INTAKE_MODE dir) {
      switch (dir) {
        case FORWARD:
          intakeMotor.set(Constants.INTAKE_FULL_SPEED);
          conveyor.set(Constants.CONVEYOR_FULL_SPEED);
          break;
        case REVERSE:
          intakeMotor.set(-Constants.INTAKE_FULL_SPEED);
          conveyor.set(-Constants.CONVEYOR_FULL_SPEED);
        case OFF:
          intakeMotor.set(0);
          conveyor.set(0);
      }
    }

    public void setIntakeOn(boolean val) {
      intakeOn = val;
    }

    public void runConveyor(boolean on) {
      conveyor.set(on ? Constants.CONVEYOR_FULL_SPEED : 0);
    }

    public void stop() {
      conveyor.set(0);
      intakeMotor.set(0);
    }

    public void putUp() {
      intakeSolLeft.set(Value.kReverse);
      intakeSolRight.set(Value.kReverse);
    }

    public void reset() {
      putUp();
      stop();
      intakeOn = false;
      intakeUp = false;
    }

    public void telemetry(NetworkTable table) {
      NetworkTable motorTable = table.getSubTable("motors");

      NetworkTable intakeMotorTable = motorTable.getSubTable("Intake Motor");
      intakeMotorTable.getEntry("name").setString("Intake Motor");
      intakeMotorTable.getEntry("loop_error").setDouble(-1);
      intakeMotorTable.getEntry("p").setDouble(-1);
      intakeMotorTable.getEntry("i").setDouble(-1);
      intakeMotorTable.getEntry("d").setDouble(-1);
      intakeMotorTable.getEntry("requested_input_speed").setDouble(-1);
      intakeMotorTable.getEntry("actual_input_speed").setDouble(-1);
      intakeMotorTable.getEntry("raw_input_speed").setDouble(intakeMotor.get());
      intakeMotorTable.getEntry("output_speed").setDouble(intakeMotor.getEncoder().getVelocity());
      intakeMotorTable.getEntry("position").setDouble(intakeMotor.getEncoder().getPosition());
      intakeMotorTable.getEntry("current").setDouble(intakeMotor.getOutputCurrent());

      NetworkTable conveyorMotorTable = motorTable.getSubTable("Conveyor Motor");
      conveyorMotorTable.getEntry("name").setString("Conveyor Motor");
      conveyorMotorTable.getEntry("loop_error").setDouble(-1);
      conveyorMotorTable.getEntry("p").setDouble(-1);
      conveyorMotorTable.getEntry("i").setDouble(-1);
      conveyorMotorTable.getEntry("d").setDouble(-1);
      conveyorMotorTable.getEntry("requested_input_speed").setDouble(-1);
      conveyorMotorTable.getEntry("actual_input_speed").setDouble(-1);
      conveyorMotorTable.getEntry("raw_input_speed").setDouble(conveyor.get());
      conveyorMotorTable.getEntry("output_speed").setDouble(conveyor.getEncoder().getVelocity());
      conveyorMotorTable.getEntry("position").setDouble(conveyor.getEncoder().getPosition());
      conveyorMotorTable.getEntry("current").setDouble(conveyor.getOutputCurrent());


      NetworkTable solenoidTable = table.getSubTable("solenoids");

      NetworkTable intakeSolLeftTable = solenoidTable.getSubTable("Left Intake Solenoid");
      intakeSolLeftTable.getEntry("name").setString("Left Intake Solenoid");
      intakeSolLeftTable.getEntry("status").setDouble(intakeSolLeft.get() == Value.kForward ? 1 : (intakeSolLeft.get() == Value.kReverse ? -1 : 0));
      
      NetworkTable intakeSolRightTable = solenoidTable.getSubTable("Right Intake Solenoid");
      intakeSolRightTable.getEntry("name").setString("Right Intake Solenoid");
      intakeSolRightTable.getEntry("status").setDouble(intakeSolRight.get() == Value.kForward ? 1 : (intakeSolRight.get() == Value.kReverse ? -1 : 0));
    }
}   

