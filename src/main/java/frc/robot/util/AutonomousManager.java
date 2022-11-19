package frc.robot.util;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.FollowTrajectoryCommand;

public class AutonomousManager {
    private TrajectoryLoader trajectoryLoader;
    private RobotContainer container;

    private NetworkTable autonomousTable;

    private NetworkTableEntry selectedAuto;

    private final String[] autoStrings = {
        "demo"
    };

    public AutonomousManager(TrajectoryLoader trajectoryLoader, RobotContainer container) {
        this.trajectoryLoader = trajectoryLoader;
        this.container = container;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        autonomousTable = inst.getTable("Autonomous");

        // Insert all of the auto options into the network tables
        autonomousTable.getEntry("autos").setStringArray(autoStrings);

        selectedAuto = autonomousTable.getEntry("selectedAuto");

        // Choose the first auto as the default
        selectedAuto.setString(autoStrings[0]);
    }

    public Command getDemo() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectoryLoader.getThreeBall());

        follow(command, trajectoryLoader.getThreeBall());
        follow(command, trajectoryLoader.getThreeBall2());

        follow(command, trajectoryLoader.getFiveBall1());
        follow(command, trajectoryLoader.getFiveBallSweep());
        follow(command, trajectoryLoader.getFiveBall2());

        return command;
    }

    private void follow(SequentialCommandGroup command, PathPlannerTrajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getSwerveDriveSubsystem(), trajectory));
    }

    private void resetRobotPose(SequentialCommandGroup command, PathPlannerTrajectory trajectory) {
        PathPlannerState initialState = trajectory.getInitialState();

        command.addCommands(new InstantCommand(() -> container
                .getSwerveDriveSubsystem()
                .resetGyroAngle(initialState.holonomicRotation.times(-1)))); // might need to reverse this angle
        command.addCommands(
                new InstantCommand(() -> container.getSwerveDriveSubsystem().resetPose(initialState.poseMeters)));
    }

    public Command loadAutonomousCommand() {
        switch (selectedAuto.getString(autoStrings[0])) {
            case "demo":
                return getDemo();
        }

        // Return an empty command group if no auto is specified
        return new SequentialCommandGroup();
    }

    public Command getAutonomousCommand() {
        return loadAutonomousCommand();
    }
}
