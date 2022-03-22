package frc.robot;

// Take in an angle in degrees and return an angular speed in rad/s
public class AngularPController {
    public static double P = 0.01;
    public static double run(double angle) {
        return P*angle;
    }
}
