package frc.robot;

/** Class with all primitives needed in the code, so if they need to be changed they can */
public class Constants {
    /** Driver station IDs of Joysticks */
    static int LEFT_JOY = 0, RIGHT_JOY = 1;
    static int CLIMBING_DRIVE_BUTTON = 2;
    static int RIGHT_DRIVER_STATION= 2;
    static int FIRST_STAGE_CLIMBING_DOWN = 3;
    static int FIRST_STAGE_CLIMBING_UP = 4;


    //Thresholds at which color sensors will detect the line
    static double BLUE_TAPE_LEFT_THRESHOLD = 50000000;
    static double RED_TAPE_LEFT_THRESHOLD = 1e8; // 2e7
    static double BLUE_TAPE_RIGHT_THRESHOLD = 27000000; 
    static double RED_TAPE_RIGHT_THRESHOLD = 1e8; 
    static double MAX_TAPE_VALUES_THRESHOLD = 400;
    /** How long the circular buffer for smoothing color sensor data is */
    static int COLOR_BUFFER_LEN = 256;

    // Auton parameters
    static double AUTON_SPEED = 0.1;
    static int AUTON_DISTANCE = 50;

    // Ports for drivetrain Talons
    static int FALCON_LF = 10;
    static int FALCON_LB = 9;
    static int FALCON_RF = 7;
    static int FALCON_RB = 8;
    static double DRIVETRAIN_MAX_SPEED = 0.75;
    static double CLIMBING_DRIVE_MAX_SPEED = 0.4;
    static double CLIMBING_MAX_SPEED = 0.7;
    static double CLIMBING_SLOW_SPEED = 0.4;
    static double JOYSTICK_CURVE_POWER = 3; // x^3 function
    static double MAX_RPM = 6380;

    // Measurements for the physical robot
    static double WHEEL_CONVERSION_FACTOR = 0.000322265628223*Math.PI;
    static double WHEEL_RAD = 0.0762;
    static double ROBOT_DIAMETER = 0.6604;
    static double WHEEL_RADPERSEC_TO_MOTOR_RPM = 3000/(11*Math.PI);

    
    // Ports for pneumatics
    static int OUTER_GEAR_LOCK_ON = 4;
    static int OUTER_GEAR_LOCK_OFF = 11;
    static int INNER_GEAR_LOCK_ON = 10;
    static int INNER_GEAR_LOCK_OFF = 5;
    static int MIDDLE_ARM_RELEASE = 0;
    static int MIDDLE_ARM_LOCK = 15;
    static int MOVE_TO_INNER_ARM = 1;
    static int MOVE_TO_OUTER_ARMS = 14;
    static int LEFT_SOL_FWD = 3;
    static int LEFT_SOL_RV = 12;
    static int RIGHT_SOL_FWD = 2;
    static int RIGHT_SOL_RV = 13;
    static int PCM_PORT = 12;

    // Port of the motor for intake, and the speed that it should run when set to max.
    static int INTAKE_MOTOR = 11;
    static int CONVEYOR_MOTOR = 3;
    static int FEEDER_MOTOR = 4;
    static int ELEVATOR_MOTOR = 6;
    static double INTAKE_FULL_SPEED = 0.8;
    static double CONVEYOR_FULL_SPEED = 0.5;
    static double FEEDER_FULL_SPEED = 0.5;
    static double ELEVATOR_FULL_SPEED = 0.5;

    // Measurements for the Limelight
    static double LIMELIGHT_HEIGHT_OF_TARGET = 103.5;
    static double LIMELIGHT_HEIGHT = 35;
    static double LIMELIGHT_ANGLE = 24.5;
    static double LIMELIGHT_HORIZONTAL_THRESHHOLD = 1;  
    static double LIMELIGHT_DEPTH_OFFSET = 2;
    static double LIMELIGHT_HORIZONTAL_OFFSET=-4.5;

    // Ports for Climbing
    static int CLIMBING_SPARK_F = 1;
    static int CLIMBING_SPARK_B = 2;
    static int FIRST_STAGE_LIMIT_SWITCH_R = 1;
    static int FIRST_STAGE_LIMIT_SWITCH_L = 0;
    static int SECOND_STAGE_LIMIT_SWITCH = 5;
    static int SHOOTER_TALON_1 = 14;
    static int SHOOTER_TALON_2 = 15;
    static double SHOOTER_SPEED = 0.55;

    // Ports and Constants for Hood
    static int HOOD_SPARK = 5;
    static int HOOD_TOP_LIMIT_SWITCH = 8;
    static int HOOD_BOTTOM_LIMIT_SWITCH = 9;
    static double HOOD_SPEED = 0.4;
    
    // Constants for Hood Auto Alligning
    static double HOOD_SPEED_SLOPE = 0.0731;
    static double HOOD_SPEED_OFFSET = 35.2;
    static double HOOD_ANGLE_SLOPE = 0.231;
    static double HOOD_ANGLE_OFFSET = 8.96;
    static double HOOD_ANGLE_CURVE = -0.000437;

    static double SHOOTER_DEFAULT_SPEED = 0.35;
    static double HOOD_DEFAULT_ANGLE = 20;

    static double LOW_GOAL_SPEED = 0.22;
    static double LOW_GOAL_ANGLE = 20;

    // P: 0.8   I: 0.02   D: 0.004   FF: 1.5
    static double kP = 0.8;
    static double kI = 0.02;
    static double kD = 0.004;
    static double kFF = 1.5;
    static double RAMP_TIME = 0.9;

    static double PID_MIN_ERR = 0.03;

    //different states for auton
    public enum AutonState {
        FIND_LINE,
        GOTO_BALL,
        PICKUP_BALL,
        TRACK_BALL,
        PREPARE_SHOOTER,
        SHOOT_BALL,
        NONE
    }

    public enum INTAKE_MODE {
        FORWARD,
        REVERSE,
        OFF
    }
    //Error level for PID to ignore DeltaT if aboce
    static double PID_ERROR = 0.1;
    

}
