package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.ArrayList;
import java.util.HashMap;

public class AutonomousManager {
    private NetworkTable autonomousTable;
    private NetworkTableEntry selectedAuto;
    private final String[] autoStrings = {"demo"};

    private HashMap<String, Command> eventMap = new HashMap<>();
    private SwerveAutoBuilder autoBuilder;

    // Load all autonomous paths
    ArrayList<PathPlannerTrajectory> demoPath = PathPlanner.loadPathGroup("demo", new PathConstraints(4, 3));

    public AutonomousManager(RobotContainer container) {
        // Allow thd custom driver station to select an auto
        initializeNetworkTablesValues();

        // Create an event map for use in all autos
        eventMap.put("print", new PrintCommand("hi"));

        SwerveDriveSubsystem swerveDriveSubsystem = container.getSwerveDriveSubsystem();

        autoBuilder = new SwerveAutoBuilder(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::setPose,
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(0.5, 0.0, 0.0),
                swerveDriveSubsystem::setVelocity,
                eventMap,
                swerveDriveSubsystem);

        // Disable at competitions
        PathPlannerServer.startServer(5811);

        // TODO
        // Robot starts thinking it is facing backwards.
    }

    private Command getPathGroupCommand(ArrayList<PathPlannerTrajectory> pathGroup) {
        return autoBuilder.fullAuto(pathGroup);
    }

    public Command getAutonomousCommand() {
        switch (selectedAuto.getString(autoStrings[0])) {
            case "demo":
                return getPathGroupCommand(demoPath);
        }

        // Return an empty command group if no auto is specified
        return new SequentialCommandGroup();
    }

    private void initializeNetworkTablesValues() {
        autonomousTable = NetworkTableInstance.getDefault().getTable("Autonomous");

        // Insert all of the auto options into the network tables
        autonomousTable.getEntry("autos").setStringArray(autoStrings);

        selectedAuto = autonomousTable.getEntry("selectedAuto");

        // Choose the first auto as the default
        selectedAuto.setString(autoStrings[0]);
    }
}
