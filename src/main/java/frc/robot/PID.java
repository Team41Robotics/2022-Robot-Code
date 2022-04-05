package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;

/**
 * Custom class to handle PID on our Falcon 500s
 */
public class PID {
    public final double SENSOR_NORM = (10.0*60)/(2048*6380);
    private boolean ready;
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double err;
    private double vel;
    private double controlSignal;
    private double acc;
    private double prevErr;
    private double rampTime;
    private double currSpeed;
    private double reqSpeed;
    private double actInputSpeed;
    private long time;
    private TalonFX motor;

    /**
     * Create a new PID controller for a Falcon 500 Motor
     * @param motor the TalonFX Motor Controller Object
     * @param kP the P gain of the controller
     * @param kI the I gain of the controller
     * @param kD the D gain of the controller
     * @param kF the Feed Forward gain of the controller
     * @param rampTime the time to take to ramp up to full speed
     */
    public PID(TalonFX motor, double kP, double kI, double kD, double kF, double rampTime) {
        this.motor = motor;
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kI = kI;
        this.rampTime = rampTime;
        ready = false;
        currSpeed = 0;
        reqSpeed = 0;
        actInputSpeed = 0;
        time = System.currentTimeMillis();
    }

    /**
     * Clear the PID to reset
     */
    public void clear() {
        ready = false;
        currSpeed = 0;
    }

    /**
     * Get the current motor output velocity
     * @return the motor velocity used for PID control
     */
    public double getVelocity() {
        return vel;
    }

    /**
     * Calculate the P section
     * @param err the current error
     * @return the adjusted value
     */
    private double p(double err) {
        return err*kP;
    }

    /**
     * Calculate the I section
     * @param err the current error
     * @param deltaT the time change since the last run of the loop
     * @return the adjusted value
     */
    private double i(double err, double deltaT) {
        acc += (err*deltaT);
        return acc * kI;
    }

    /**
     * Calculate the D section
     * @param err the current error
     * @param deltaT the time change since the last run of the loop
     * @return the adjusted value
     */
    private double d(double err, double deltaT) {
        double diff = err-prevErr;
        return (diff/deltaT)*kD;
    }

    /**
     * Run a full loop of PID
     * @param speed the desired motor output speed [-1, 1]
     */
    public void run(double speed) {
        reqSpeed = speed;
        double deltaSpeed;
        double usedDeltaSpeed;
        long currentTime = System.currentTimeMillis();
        double deltaT = (currentTime-time)/1000.0;
        double reqDeltaSpeed = speed-currSpeed;
        time = currentTime;
        //throws out deltaT if it is too large, happens during startup
        if (deltaT > Constants.PID_ERROR) {
            System.out.println("deltaT is too big");
            return;
        }

        if (rampTime == 0) {
            deltaSpeed = Double.MAX_VALUE; // It won't get chosen over set speed
        } else {
            deltaSpeed = deltaT/rampTime;
        }

        if (reqDeltaSpeed < 0) {
            if (Math.abs(reqDeltaSpeed) > deltaSpeed) {
                usedDeltaSpeed = -deltaSpeed;
            } else {
                usedDeltaSpeed = reqDeltaSpeed;
            }
        } else {
            usedDeltaSpeed = Math.min(reqDeltaSpeed, deltaSpeed);
        }

        runCore(currSpeed+usedDeltaSpeed, deltaT);
        time = System.currentTimeMillis();
    }

    /**
     * Run the core of the PID
     * @param s the speed to calculate from
     * @param deltaT the change in time since last run of the loop
     */
    private void runCore(double s, double deltaT) {
        actInputSpeed = s;
        currSpeed = s;
        vel = motor.getSelectedSensorVelocity()*SENSOR_NORM;
        err = currSpeed-vel;

        if (!ready) {
            prevErr = err;
            time = System.currentTimeMillis();
            ready = true;
            acc = 0;
            return;
        }

        controlSignal = p(err)+i(err, deltaT)+d(err, deltaT);
        controlSignal += currSpeed*kF;
        motor.set(ControlMode.PercentOutput, controlSignal);
    }

    /**
     * Run the PID loop, but without the ramp up/down - NOT A GOOD IDEA FOR HIGH CHANGES IN SPEED
     * @param speed the desired motor speed
     */
    public void runNoRamp(double speed) {
        reqSpeed = speed;
        long currentTime = System.currentTimeMillis();
        double deltaT = (currentTime-time)/1000.0;
        
        runCore(speed, deltaT);
        time = System.currentTimeMillis();
    }

    /**
     * Get the current PID error
     * @return the current error
     */
    public double getError() {
        return err;
    }

    /**
     * Get the current motor current
     * @return the current in amps
     */
    public double getCurrent(){
        return motor.getSupplyCurrent();
    }

    /**
     * Gets the signal that the controller is currently sending to the motor
     * @return the control signal [-1, 1]
     */
    public double getControlSignal() {
        return controlSignal;
    }

    /**
     * Checks if the PID is within the minimum error threshold
     * @return Whether the PID has basically reached its target
     */
    public boolean isReady() {
        return err < Constants.PID_MIN_ERR;
    }

    /**
     * Upload all telemetry data for this motor
     * @param table the telemetry motor subtable
     * @param name the name of this PID controller
     */
    public void telemetry(NetworkTable table, String name) {
        NetworkTable currentTable = table.getSubTable(name);

        currentTable.getEntry("name").setString(name);
        currentTable.getEntry("loop_error").setDouble(err);
        currentTable.getEntry("p").setDouble(kP);
        currentTable.getEntry("i").setDouble(kI);
        currentTable.getEntry("d").setDouble(kD);
        currentTable.getEntry("requested_input_speed").setDouble(reqSpeed);
        currentTable.getEntry("actual_input_speed").setDouble(actInputSpeed);
        currentTable.getEntry("raw_input_speed").setDouble(getControlSignal());
        currentTable.getEntry("output_speed").setDouble(motor.getSelectedSensorVelocity());
        currentTable.getEntry("position").setDouble(motor.getSelectedSensorPosition());
        currentTable.getEntry("current").setDouble(motor.getSupplyCurrent());
    }
}
