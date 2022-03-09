package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class PID {
    public final double SENSOR_NORM = (10.0*60)/(2048*6380);
    private TalonFX motor;
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double err;
    private double vel;
    private double controlSignal;
    private long time;
    private boolean ready;
    private double acc;
    private double prevErr;
    private double rampTime;
    private double currSpeed;

    public PID(TalonFX motor, double kP, double kI, double kD, double kF, double rampTime) {
        this.motor = motor;
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kI = kI;
        this.rampTime = rampTime;
        ready = false;
        currSpeed = 0;
        time = System.currentTimeMillis();
    }

    public void clear() {
        ready = false;
        currSpeed = 0;
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

    public void run(double speed) {
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

    private void runCore(double s, double deltaT) {
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

    public void runNoRamp(double speed) {
        long currentTime = System.currentTimeMillis();
        double deltaT = (currentTime-time)/1000.0;
        runCore(speed, deltaT);
        time = System.currentTimeMillis();
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
}
