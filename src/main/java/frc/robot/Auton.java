package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.AutonState;
import frc.robot.Constants.INTAKE_MODE;

public class Auton {
    private static double robotPos;
    private static boolean thirdBallClose;
    private static long autonHumanStationWait, autonShootingStartTime;
    private static DigitalInput beamBreak;

    /**
     * Initialize auton variables
     * @param beamSensor the beam sensor input to use for ball sensing
     */
    public static void init(DigitalInput beamSensor) {
        robotPos = 0;
        autonHumanStationWait = 0;
        autonShootingStartTime = 0;
        thirdBallClose = false;
        beamBreak = beamSensor;
    }

    /**
     * Line the robot up with the closest ball of the correct color
     * @return Either current or next state, depending on readiness
     */
    public static AutonState alignToBall() {
        Shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
        Hood.home();
        return (ballAlign() ? AutonState.GOTO_BALL : AutonState.ALIGN_TO_BALL);
    }

    /**
     * Move towards the ball until it is intook
     * @return Either current or next state, depending on readiness
     */
    public static AutonState gotoBall() {
        Shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
        Hood.home();
        Limelight.setLedOn(true);
        // Beam break here
        if (!beamBreak.get()) {
          do {
            Drivetrain.setPosition(0);
          } while (Drivetrain.getPosition() > 1);
          return AutonState.TRACK_GOAL;
        }
        double angle = PhotonCamera.getYaw();
        Drivetrain.runInverseKinematics(Constants.BALL_FOLLOWING_kP * -angle, Constants.AUTON_SPEED_M_PER_S * (2.0 / 3));
        return AutonState.GOTO_BALL;
    }

    /**
     * Move past the detected ball to insure it is picked up
     * @deprecated
     * @return Either current or next state, depending on readiness
     */
    public static AutonState pickupBall() {
        Shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
        Hood.home();
        System.out.println(Drivetrain.getPosition());
        if (Drivetrain.getPosition() >= Constants.BALL_DISTANCE_FROM_BOT) {
          Drivetrain.setNoRamp(0);
          // intake.run(INTAKE_MODE.OFF);
          return AutonState.TRACK_GOAL;
        } else {
          Drivetrain.set(Constants.AUTON_SPEED);
        }
        return AutonState.PICKUP_BALL;
    }

    /**
     * Align the robot towards the hub
     * @return Either current or next state, depending on readiness
     */
    public static AutonState trackGoal() {
        Shooter.setSpeed(Constants.SHOOTER_DEFAULT_SPEED);
        Hood.home();
        return (goalAlign() ? AutonState.PREPARE_SHOOTER : AutonState.TRACK_GOAL);
    }

    /**
     * Set the shooter and hood to correct settings
     * @return Either current or next state, depending on readiness
     */
    public static AutonState prepareShooter() {
        setupShoot();
        if (Hood.isReady()) {
          autonShootingStartTime = System.currentTimeMillis();
          return AutonState.SHOOT_BALL;
        }
        Drivetrain.setNoRamp(0);
        return AutonState.PREPARE_SHOOTER;
    }

    /**
     * Launch the balls from the robot
     * @return Either current or next state, depending on readiness
     */
    public static AutonState shootBall() {
        return (shoot() ? AutonState.ALIGN_TO_THIRD_BALL : AutonState.SHOOT_BALL);
    }

    /**
     * Line up the robot with the closest ball of the correct color (by the player station)
     * @return Either current or next state, depending on readiness
     */
    public static AutonState alignToThirdBall() {
        return (ballAlign() ? AutonState.GOTO_THIRD_BALL : AutonState.ALIGN_TO_THIRD_BALL);
    }

    /**
     * Move towards the third ball, slowing down as it gets closer, until its picked up
     * @return Either current or next state, depending on readiness
     */
    public static AutonState gotoThirdBall() {
        if (Drivetrain.getDanger()){
            return AutonState.WAIT_FOR_BALL;
        }
        if (!beamBreak.get()) {
            Drivetrain.setNoRamp(0);
            autonHumanStationWait = System.currentTimeMillis();
            robotPos = Drivetrain.getPosition();
            return AutonState.WAIT_FOR_BALL;
        } else {
            if (PhotonCamera.getArea() > 5) {
                thirdBallClose = true;
            }
            double angle = PhotonCamera.getYaw();
            double slowSpeed = 0.5;
            Drivetrain.runInverseKinematics((Constants.BALL_FOLLOWING_kP * -angle),
                !thirdBallClose ? Constants.AUTON_SPEED_M_PER_S : slowSpeed);
        }
        return AutonState.GOTO_THIRD_BALL;
    }

    /**
     * Wait a sec for the human player to feed another ball
     * @return Either current or next state, depending on readiness
     */
    public static AutonState waitForBall() {
        if (System.currentTimeMillis() - autonHumanStationWait > Constants.AUTON_HUMAN_BALL_WAIT_TIME) {
            return AutonState.MOVE_TOWARDS_GOAL;
          } else {
            Drivetrain.setNoRamp(0);
          }
          return AutonState.WAIT_FOR_BALL;
    }

    /**
     * Move a little closer to the hub to ensure a more accurate shot
     * @return Either current or next state, depending on readiness
     */
    public static AutonState moveTowardsGoal() {
        if (Math.abs(Drivetrain.getPosition() - robotPos) >= Constants.DISTANCE_FROM_HUMAN_STATION) {
            do
            Drivetrain.stop();
            while (!Drivetrain.isReady());
            return AutonState.ALIGN_TO_GOAL_AGAIN;
          } else {
            Drivetrain.runInverseKinematics(0.075, -Constants.AUTON_SPEED_M_PER_S);
          }
          return AutonState.MOVE_TOWARDS_GOAL;
    }

    /**
     * Align the robot to the hub in preparation to shoot
     * @return Either current or next state, depending on readiness
     */
    public static AutonState alignToGoalAgain() {
        return (goalAlign() ? AutonState.SHOOT_AGAIN : AutonState.ALIGN_TO_GOAL_AGAIN);
    }

    /**
     * Shoot the balls a second time
     * @return Either current or next state, depending on readiness
     */
    public static void shootAgain() {
        setupShoot();
        if (Hood.isReady()) {
          autonShootingStartTime = System.currentTimeMillis();
          if (shoot()) {
            Limelight.setLedOn(false);
          }
        }
        Drivetrain.setNoRamp(0);
        return;
    }

    /**
     * Don't do anything
     */
    public static void none() {
        Drivetrain.setNoRamp(0);
    }

    /**
     * Align the robot to the closest ball
     * @return Whether the robot is done aligning
     */
    private static boolean ballAlign() {
        if (Drivetrain.alignToBall()) {
            Drivetrain.setNoRamp(0);
            Intake.run(INTAKE_MODE.FORWARD);
            return true;
        }
        return false;
    }

    /**
     * Align the robot to the goal
     * @return Whether the robot is aligned
     */
    private static boolean goalAlign() {
        return Drivetrain.alignToGoal();
    }

    /**
     * Gather all data needed and set shooter speed and hood position
     */
    private static void setupShoot() {
        double distance = Limelight.estimateDistance();
        double speed = (distance * Constants.HOOD_SPEED_SLOPE) + Constants.HOOD_SPEED_OFFSET;
        double angle = (distance * distance * Constants.HOOD_ANGLE_CURVE) + (distance * Constants.HOOD_ANGLE_SLOPE)
            + Constants.HOOD_ANGLE_OFFSET;
        Shooter.setSpeed(speed / 100);
        Hood.setToPosition(angle);
    }

    /**
     * Shoot a ball
     * @return whether the ball has been shot or not
     */
    private static boolean shoot() {
        Shooter.runFeeder(true);
        Shooter.runElevator(Constants.ELEVATOR_FULL_SPEED);
        Intake.runConveyor(true);
        if (System.currentTimeMillis() - autonShootingStartTime >= Constants.AUTON_SHOOTER_WAIT_TIME) {
          Shooter.runFeeder(false);
          Shooter.runElevator(0);
          Intake.runConveyor(false);
          return true;
        }
        return false;
    }
}
