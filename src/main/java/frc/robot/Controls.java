package frc.robot;

public class Controls {
    public static class SecondDriverStation {
        public static int INCREASE_HOOD_OFFSET = 11;
        public static int DECREASE_HOOD_OFFSET = 12;

        public static int MANUAL_SHOOTER_SPEED = 14;
        public static int SHOOTER_SPEED_SLIDER = 0;
        public static int LOW_GOAL_SETUP = 5;
        public static int AUTO_SHOOTING = 6;
        public static int SHOOTER_WARMUP = 15;

        public static int MANUAL_CLIMBING_TOGGLE = 13;
        public static int ENABLE_PISTON_BRAKE = 14;
        public static int CLIMBING_STATE_POV = 1;

        public static int HOOD_UP = 7;
        public static int HOOD_DOWN = 8;

        public static int SHOOTER_OFFSET_UP = 9;
        public static int SHOOTER_OFFSET_DOWN = 10;

        public static int FEED_BALL_TO_SHOOTER = 1;
        public static int ZERO_HOOD = 2;
    }

    public static class LeftJoy {
        public static int CLIMB_FWD = 3;
        public static int CLIMB_RV = 4;

        public static int INTAKE_PISTON_TOGGLE = 1;
    }

    public static class RightJoy {
        public static int INTAKE_TOGGLE = 1;
        public static int INTAKE_REVERSE = 3;
    }
}
