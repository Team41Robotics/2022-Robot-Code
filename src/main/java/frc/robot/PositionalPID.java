package frc.robot;

/**
 * Custom class to manage a positional P system
 */
public class PositionalPID {
    private boolean ready;
    private double kP;
    private double err;
    private double controlSignal;
    private long time;

    /**
     * Create a positional P controller
     * @param kP the P gain of the controller
     */
    public PositionalPID(double kP) {
        this.kP = kP;
        ready = false;
        time = System.currentTimeMillis();
    }

    /**
     * Run the P section of the controller
     * @param err the current error
     * @return the adjusted output value
     */
    private double p(double err) {
        return err*kP;
    }

    /**
     * Run the core of this P controller
     * @param deltaT the change in time since the last run of the loop
     * @param pos the current position
     * @return the signal to send to the motors
     */
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

    /**
     * Run the full P controller
     * @param pos the current position
     * @return the signal to send
     */
    public double run(double pos) {
        long currentTime = System.currentTimeMillis();
        double deltaT = (currentTime-time)/1000.0;
        double result = runCore(deltaT, pos);
        time = System.currentTimeMillis();
        return result;
    }

    /**
     * Get the error of this P controller
     * @return the current erro
     */
    public double getError() {
        return err;
    }
}
