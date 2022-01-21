package frc.robot;

public class Constants {
    static int LEFT_JOY = 1;
    static int RIGHT_JOY = 0;

    static int BLUE_TAPE_THRESHOLD = 950;
    static int RED_TAPE_THRESHOLD = 1000;

    static double AUTON_SPEED = -0.1;
    static int AUTON_DISTANCE = -10;

    static int SPARK_LF = 15;
    static int SPARK_LB = 1;
    static int SPARK_RF = 5;
    static int SPARK_RB = 4;

    static int LEFT_SOL_FWD = 3;
    static int LEFT_SOL_RV = 2;
    static int RIGHT_SOL_FWD = 5;
    static int RIGHT_SOL_RV = 4;
    static int PCM_PORT = 15;

    static int INTAKE_MOTOR = 14;
    static double INTAKE_FULL_SPEED = .6;

    static int LIMELIGHT_HEIGHT_OF_TARGET = 71;
    static int LIMELIGHT_HEIGHT = 32;
    static double LIMELIGHT_ANGLE = 0;

    static double WHEEL_CONVERSION_FACTOR = Math.PI/2;
    
    public enum AutonState {
        FIND_LINE,
        GOTO_BALL,
        PICKUP_BALL
    }
}
