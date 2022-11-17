package frc.robot.util;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class TrajectoryLoader {
    private static final String TWO_BALL_PATH = "twoball";
    private static final String TWO_BALL_FAR_PATH = "twoball far new";
    private static final String TWO_BALL_FAR_MANUAL_GYRO_PATH = "threeball";
    private static final String THREE_BALL_PATH = "threeball";
    private static final String THREE_BALL_2_PATH = "threeball 2";
    private static final String FIVE_BALL_PATH = "fiveball alt";
    private static final String FIVE_BALL_1_PATH = "fiveball 1";
    private static final String FIVE_BALL_SWEEP_PATH = "fiveball sweep";
    private static final String FIVE_BALL_2_PATH = "fiveball 2";
    private static final String FOUR_BALL_PATH = "fourball";
    private static final String FOUR_BALL_2_PATH = "fourball 2";
    private static final String FOUR_BALL_3_PATH = "fourball 3";
    private static final String TWO_BALL_STEAL_PATH = "twoball far steal";
    private static final String ONE_BALL_STEAL_PATH = "oneball far steal";

    private PathPlannerTrajectory threeBall;
    private PathPlannerTrajectory threeBall2;
    private PathPlannerTrajectory twoBall;
    private PathPlannerTrajectory twoBallFar;
    private PathPlannerTrajectory twoBallFarManualGyro;
    private PathPlannerTrajectory fourBall;
    private PathPlannerTrajectory fourBall2;
    private PathPlannerTrajectory fourBall3;
    private PathPlannerTrajectory fiveBall;
    private PathPlannerTrajectory fiveBall1;
    private PathPlannerTrajectory fiveBallSweep;
    private PathPlannerTrajectory fiveBall2;
    private PathPlannerTrajectory twoBallSteal;
    private PathPlannerTrajectory oneBallSteal;

    public TrajectoryLoader() {
        threeBall = loadTrajectory(THREE_BALL_PATH, 5, 6.0);
        threeBall2 = loadTrajectory(THREE_BALL_2_PATH, 5, 2.5);
        // twoBall = loadTrajectory(TWO_BALL_PATH, 5, 2.0);
        twoBallFar = loadTrajectory(THREE_BALL_PATH, 5, 4.0);
        // twoBallFarManualGyro = loadTrajectory(TWO_BALL_FAR_MANUAL_GYRO_PATH, 5, 2.5);
        // fiveBall = loadTrajectory(FIVE_BALL_PATH, 5, 2.5);
        fourBall = loadTrajectory(FOUR_BALL_PATH, 5, 2.5);
        fourBall2 = loadTrajectory(FOUR_BALL_2_PATH, 5, 6.0);
        fourBall3 = loadTrajectory(FOUR_BALL_3_PATH, 5, 6.0);
        fiveBall1 = loadTrajectory(FIVE_BALL_1_PATH, 5, 5.5);
        fiveBallSweep = loadTrajectory(FIVE_BALL_SWEEP_PATH, 2.5, 3.5);
        fiveBall2 = loadTrajectory(FIVE_BALL_2_PATH, 5, 6.0);
        // twoBallSteal = loadTrajectory(TWO_BALL_STEAL_PATH, 5, 2.5);
        // oneBallSteal = loadTrajectory(ONE_BALL_STEAL_PATH, 5, 2.5);
    }

    private PathPlannerTrajectory loadTrajectory(String path, double maxVel, double maxAccel) {
        return PathPlanner.loadPath(path, maxVel, maxAccel);
    }

    public PathPlannerTrajectory getThreeBall() {
        return threeBall;
    }

    public PathPlannerTrajectory getThreeBall2() {
        return threeBall2;
    }

    public PathPlannerTrajectory getTwoBall() {
        return twoBall;
    }

    public PathPlannerTrajectory getTwoBallFar() {
        return twoBallFar;
    }

    public PathPlannerTrajectory getTwoBallFarManualGyro() {
        return twoBallFarManualGyro;
    }

    public PathPlannerTrajectory getFiveBall() {
        return fiveBall;
    }

    public PathPlannerTrajectory getFiveBall1() {
        return fiveBall1;
    }

    public PathPlannerTrajectory getFiveBallSweep() {
        return fiveBallSweep;
    }

    public PathPlannerTrajectory getFiveBall2() {
        return fiveBall2;
    }

    public PathPlannerTrajectory getFourBall() {
        return fourBall;
    }

    public PathPlannerTrajectory getFourBall2() {
        return fourBall2;
    }

    public PathPlannerTrajectory getFourBall3() {
        return fourBall3;
    }

    public PathPlannerTrajectory getTwoBallSteal() {
        return twoBallSteal;
    }

    public PathPlannerTrajectory getOneBallSteal() {
        return oneBallSteal;
    }
}
