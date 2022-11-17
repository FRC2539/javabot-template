package frc.robot.util;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.control.SwerveDriveSignal;
import java.util.Optional;
import java.util.function.Supplier;

public class TrajectoryFollower {
    private HolonomicDriveController driveController;

    private PathPlannerTrajectory currentTrajectory = null;

    private Supplier<Rotation2d> desiredRotation = null;

    private boolean isPathPlanner = false;

    private PathPlannerTrajectory.PathPlannerState lastState = null;

    private double startTime = Double.NaN;

    private boolean finished = false;

    public TrajectoryFollower(
            PIDController forwardController, PIDController strafeController, ProfiledPIDController rotationController) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        driveController = new HolonomicDriveController(forwardController, strafeController, rotationController);
    }

    public Optional<SwerveDriveSignal> update(Pose2d currentPose) {
        double timeSinceStart;

        if (currentTrajectory == null) {
            return Optional.empty();
        }

        // If the trajectory has not been started, update the start time and reset the follower state
        if (Double.isNaN(startTime)) {
            startTime = Timer.getFPGATimestamp();
            reset();
        } else if (isFinished()) {
            currentTrajectory = null;
            this.desiredRotation = null;
            return Optional.empty();
        }

        timeSinceStart = Timer.getFPGATimestamp() - startTime;

        SwerveDriveSignal signal;

        if (isPathPlanner) {
            signal = calculateDriveSignal(currentPose, (PathPlannerTrajectory) currentTrajectory, timeSinceStart);
        } else {
            signal = calculateDriveSignal(currentPose, currentTrajectory, desiredRotation, timeSinceStart);
        }

        return Optional.of(signal);
    }

    private SwerveDriveSignal calculateDriveSignal(
            Pose2d currentPose, PathPlannerTrajectory trajectory, Supplier<Rotation2d> desiredRotation, double timeSinceStart) {
        if (timeSinceStart > trajectory.getTotalTimeSeconds()) {
            finished = true;
            return new SwerveDriveSignal();
        }

        lastState = (PathPlannerState) trajectory.sample(timeSinceStart);

        return new SwerveDriveSignal(driveController.calculate(currentPose, lastState, desiredRotation.get()), false);
    }

    private SwerveDriveSignal calculateDriveSignal(
            Pose2d currentPose, PathPlannerTrajectory trajectory, double timeSinceStart) {
        if (timeSinceStart > trajectory.getTotalTimeSeconds()) {
            finished = true;
            return new SwerveDriveSignal();
        }

        lastState = (PathPlannerState) trajectory.sample(timeSinceStart);

        return new SwerveDriveSignal(
                driveController.calculate(currentPose, lastState, lastState.holonomicRotation),
                false);
    }

    public boolean isPathPlanner() {
        return isPathPlanner;
    }

    public PathPlannerTrajectory.State getLastState() {
        return lastState;
    }

    public boolean isFinished() {
        return finished;
    }

    public void reset() {
        finished = false;
    }

    public void cancel() {
        currentTrajectory = null;
    }

    public void follow(PathPlannerTrajectory trajectory, Supplier<Rotation2d> desiredRotation) {
        currentTrajectory = trajectory;
        this.desiredRotation = desiredRotation;
        startTime = Double.NaN;
        isPathPlanner = false;
    }

    public void follow(PathPlannerTrajectory trajectory) {
        currentTrajectory = trajectory;
        startTime = Double.NaN;
        isPathPlanner = true;
    }

    public Optional<PathPlannerTrajectory> getCurrentTrajectory() {
        return Optional.ofNullable(currentTrajectory);
    }
}
