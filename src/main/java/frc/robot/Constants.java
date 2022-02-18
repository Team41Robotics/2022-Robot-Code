package frc.robot;

/** Class with all primitives needed in the code, so if they need to be changed they can */
public class Constants {
    /** Driver station IDs of Joysticks */
    static int LEFT_JOY = 0, RIGHT_JOY = 1;
    static int CLIMBING_DRIVE_BUTTON = 2;
    static int FIRST_STAGE_CLIMBING_DOWN = 3;
    static int FIRST_STAGE_CLIMBING_UP = 4;


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
    static int AUTON_DISTANCE = -20;

    // Ports for drivetrain Talons
    static int FALCON_LF = 10;
    static int FALCON_LB = 9;
    static int FALCON_RF = 7;
    static int FALCON_RB = 8;
    static double DRIVETRAIN_MAX_SPEED = 0.85;
    static double CLIMBING_DRIVE_MAX_SPEED = 0.4;
    static double CLIMBING_SPEED = 0.7;
    static double JOYSTICK_CURVE_POWER = 3; // x^3 function
    static double MAX_RPM = 6380;

    // Measurements for the physical robot
    static double WHEEL_CONVERSION_FACTOR = Math.PI/2;
    static double WHEEL_RAD = 0.0762;
    static double ROBOT_DIAMETER = 0.6604;
    static double WHEEL_RADPERSEC_TO_MOTOR_RPM = 3000/(11*Math.PI);

    
    // Ports for pneumatics
    static int LEFT_SOL_FWD = 3;
    static int LEFT_SOL_RV = 2;
    static int RIGHT_SOL_FWD = 5;
    static int RIGHT_SOL_RV = 4;
    static int PCM_PORT = 12;

    // Port of the motor for intake, and the speed that it should run when set to max.
    static int INTAKE_MOTOR = 11;
    static double INTAKE_FULL_SPEED = .6;

    // Measurements for the Limelight
    static double LIMELIGHT_HEIGHT_OF_TARGET = 49.5;
    static double LIMELIGHT_HEIGHT = 24.75;
    static double LIMELIGHT_ANGLE = 6;
    static double LIMELIGHT_HORIZONTAL_THRESHHOLD = 2;  

    static int FIRST_STAGE_SPARK_F = 1;
    static int FIRST_STAGE_SPARK_B = 2;


    //different states for auton
    public enum AutonState {
        FIND_LINE,
        GOTO_BALL,
        PICKUP_BALL,
        TRACK_BALL,
        NONE
    }
    //Error level for PID to ignore DeltaT if aboce
    static double PID_ERROR = 0.1;
    

}
