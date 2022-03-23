package frc.robot;

import edu.wpi.first.networktables.NetworkTable;

public class PositionalPID {
    public final double SENSOR_NORM = (10.0*60)/(2048*6380);
    private boolean ready;
    private double kP;
    private double err;
    private double controlSignal;
    private double reqSpeed;
    private double actInputSpeed;
    private long time;

    public PositionalPID(double kP) {
        this.kP = kP;
        ready = false;
        reqSpeed = 0;
        actInputSpeed = 0;
        time = System.currentTimeMillis();
    }

    private double p(double err) {
        return err*kP;
    }

    private double runCore(double deltaT, double pos) {
        err = pos;

        if (!ready) {
            time = System.currentTimeMillis();
            ready = true;
            return 0;
        }

        controlSignal = p(err);
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

    public void telemetry(NetworkTable table, String name) {
        NetworkTable currentTable = table.getSubTable(name);

        currentTable.getEntry("name").setString(name);
        currentTable.getEntry("loop_error").setDouble(getError());
        currentTable.getEntry("p").setDouble(kP);
        currentTable.getEntry("requested_input_speed").setDouble(reqSpeed);
        currentTable.getEntry("actual_input_speed").setDouble(actInputSpeed);
        currentTable.getEntry("raw_input_speed").setDouble(getControlSignal());
    }
}
