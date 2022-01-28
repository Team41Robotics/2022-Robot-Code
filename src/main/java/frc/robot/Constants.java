package frc.robot;

public class Constants {
    //Positions Of Joysticks
    static int LEFT_JOY = 1;
    static int RIGHT_JOY = 0;

    //Thresholds at which color sensors will detect the line
    static double BLUE_TAPE_LEFT_THRESHOLD = 50000000;
    static double RED_TAPE_LEFT_THRESHOLD = 1e8; // 2e7
    static double BLUE_TAPE_RIGHT_THRESHOLD = 27000000; 
    static double RED_TAPE_RIGHT_THRESHOLD = 1e8; 
    static double MAX_TAPE_VALUES_THRESHOLD = 100;
    //How long the circular buffer for smoothing color sensor data is
    static int COLOR_BUFFER_LEN = 256;
    //1.5 e 7 tested as of 1/25/22 suggested by hadzic 200 sec delay before test
    static double AUTON_SPEED = -0.1;
    static int AUTON_DISTANCE = 0;
    //ports of the 4 motor control sparks
    static int SPARK_LF = 15;
    static int SPARK_LB = 1;
    static int SPARK_RF = 5;
    static int SPARK_RB = 4;
    
    static int LEFT_SOL_FWD = 3;
    static int LEFT_SOL_RV = 2;
    static int RIGHT_SOL_FWD = 5;
    static int RIGHT_SOL_RV = 4;
    static int PCM_PORT = 15;
    //port of the motor for intake, and the speed that it should run when set to max.
    static int INTAKE_MOTOR = 14;
    static double INTAKE_FULL_SPEED = .6;
    //how high the indicators for the limelight are, the height of the the limelight, and angle
    static int LIMELIGHT_HEIGHT_OF_TARGET = 50;
    static int LIMELIGHT_HEIGHT = 22;
    static double LIMELIGHT_ANGLE = 60;
    //self explanitory
    static double WHEEL_CONVERSION_FACTOR = Math.PI/2;
    
    static double LIME_LITE_THRESH_HOLD = 5;    
    //different states for auton
    public enum AutonState {
        FIND_LINE,
        GOTO_BALL,
        PICKUP_BALL,
        TARACK_BALL
    }
}
