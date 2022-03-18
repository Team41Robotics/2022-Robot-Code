package frc.robot;

import edu.wpi.first.networktables.NetworkTable;

public class PositionalPID {
    public final double SENSOR_NORM = (10.0*60)/(2048*6380);
    private boolean ready;
    private double kP;
    private double kI;
    private double kD;
    private double err;
    private double vel;
    private double controlSignal;
    private double acc;
    private double prevErr;
    private double reqSpeed;
    private double actInputSpeed;
    private long time;

    public PositionalPID(double kP, double kI, double kD, double kF, double rampTime) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        ready = false;
        reqSpeed = 0;
        actInputSpeed = 0;
        time = System.currentTimeMillis();
    }

    public void clear() {
        ready = false;
    }

    public double getVelocity() {
        return vel;
    }

    private double p(double err) {
        return err*kP;
    }

    private double i(double err, double deltaT) {
        acc += (err*deltaT);
        return acc * kI;
    }

    private double d(double err, double deltaT) {
        double diff = err-prevErr;
        return (diff/deltaT)*kD;
    }

    private double runCore(double deltaT, double pos) {
        err = pos;

        if (!ready) {
            prevErr = err;
            time = System.currentTimeMillis();
            ready = true;
            acc = 0;
            return 0;
        }

        controlSignal = p(err)+i(err, deltaT)+d(err, deltaT);
        return controlSignal;
    }

    public double run(double pos) {
        long currentTime = System.currentTimeMillis();
        double deltaT = (currentTime-time)/1000.0;
        double result = runCore(deltaT, pos);
        time = System.currentTimeMillis();
        return result;
    }

    public double getError() {
        return err;
    }

    public double getControlSignal() {
        return controlSignal;
    }

    public boolean isReady() {
        return err < Constants.PID_MIN_ERR;
    }

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
    }
}
