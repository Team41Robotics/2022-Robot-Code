package frc.robot;

/** Class with all primitives needed in the code, so if they need to be changed they can */
public class Constants {
    /** Driver station IDs of Joysticks */
    static int LEFT_JOY = 1, RIGHT_JOY = 0;

    //Thresholds at which color sensors will detect the line
    static double BLUE_TAPE_LEFT_THRESHOLD = 50000000;
    static double RED_TAPE_LEFT_THRESHOLD = 1e8; // 2e7
    static double BLUE_TAPE_RIGHT_THRESHOLD = 27000000; 
    static double RED_TAPE_RIGHT_THRESHOLD = 1e8; 
    static double MAX_TAPE_VALUES_THRESHOLD = 100;
    /** How long the circular buffer for smoothing color sensor data is */
    static int COLOR_BUFFER_LEN = 256;

    // Auton parameters
    static double AUTON_SPEED = -0.1;
    static int AUTON_DISTANCE = 0;

    // Ports of the 4 sparks
    static int SPARK_LF = 15;
    static int SPARK_LB = 1;
    static int SPARK_RF = 5;
    static int SPARK_RB = 4;

    // Measurements for the physical robot
    static double WHEEL_CONVERSION_FACTOR = Math.PI/2;
    
    // Ports for pneumatics
    static int LEFT_SOL_FWD = 3;
    static int LEFT_SOL_RV = 2;
    static int RIGHT_SOL_FWD = 5;
    static int RIGHT_SOL_RV = 4;
    static int PCM_PORT = 15;

    // Port of the motor for intake, and the speed that it should run when set to max.
    static int INTAKE_MOTOR = 14;
    static double INTAKE_FULL_SPEED = .6;

    // Measurements for the Limelight
    static int LIMELIGHT_HEIGHT_OF_TARGET = 50;
    static int LIMELIGHT_HEIGHT = 22;
    static double LIMELIGHT_ANGLE = 60;
    static double LIMELIGHT_HORIZONTAL_THRESHHOLD = 5;  

    //different states for auton
    public enum AutonState {
        FIND_LINE,
        GOTO_BALL,
        PICKUP_BALL,
        TRACK_BALL
    }
}
