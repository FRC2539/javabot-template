package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.function.Supplier;

public class FollowTrajectoryCommand extends CommandBase {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private PathPlannerTrajectory trajectory;
    private Supplier<Rotation2d> desiredRotation;
    private boolean isPathPlanner;

    public FollowTrajectoryCommand(
            SwerveDriveSubsystem swerveDriveSubsystem,
            PathPlannerTrajectory trajectory,
            Supplier<Rotation2d> desiredRotation) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.trajectory = trajectory;
        this.desiredRotation = desiredRotation;

        isPathPlanner = false;

        addRequirements(swerveDriveSubsystem);
    }

    public FollowTrajectoryCommand(SwerveDriveSubsystem swerveDriveSubsystem, PathPlannerTrajectory trajectory) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.trajectory = trajectory;
        this.desiredRotation = null;

        isPathPlanner = true;

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        if (isPathPlanner) swerveDriveSubsystem.getFollower().follow(trajectory);
        else swerveDriveSubsystem.getFollower().follow(trajectory, desiredRotation);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.getFollower().cancel();
    }

    @Override
    public boolean isFinished() {
        return swerveDriveSubsystem.getFollower().getCurrentTrajectory().isEmpty();
    }
}
