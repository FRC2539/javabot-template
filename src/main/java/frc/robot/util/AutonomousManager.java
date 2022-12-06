package frc.robot.util;

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
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.ArrayList;
import java.util.HashMap;

public class AutonomousManager {
    private RobotContainer container;

    private NetworkTable autonomousTable;

    private NetworkTableEntry selectedAuto;

    private final String[] autoStrings = {"demo"};

    private HashMap<String, Command> eventMap = new HashMap<>();

    private SwerveAutoBuilder autoBuilder;

    public AutonomousManager(RobotContainer container) {
        this.container = container;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        autonomousTable = inst.getTable("Autonomous");

        // Insert all of the auto options into the network tables
        autonomousTable.getEntry("autos").setStringArray(autoStrings);

        selectedAuto = autonomousTable.getEntry("selectedAuto");

        // Choose the first auto as the default
        selectedAuto.setString(autoStrings[0]);

        eventMap.put("print", new PrintCommand("hi"));
        
        // TODO
        // Robot starts thinking it is facing backwards.

        SwerveDriveSubsystem swerveDriveSubsystem = container.getSwerveDriveSubsystem();

        autoBuilder = new SwerveAutoBuilder(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::setPose,
                Constants.SwerveConstants.swerveKinematics,
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(0.5, 0.0, 0.0),
                swerveDriveSubsystem::setModuleStatesProxy,
                eventMap,
                swerveDriveSubsystem);

        PathPlannerServer.startServer(5811);
    }

    public Command getDemo() {
        return getPathGroupCommand("demo");
    }

    private Command getPathGroupCommand(String autoName) {
        ArrayList<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(autoName, new PathConstraints(4, 3));
        return autoBuilder.fullAuto(pathGroup);
    }

    public Command loadAutonomousCommand() {
        switch (selectedAuto.getString(autoStrings[0])) {
            case "demo":
                return getDemo();
        }

        return getPathGroupCommand(selectedAuto.getString(autoStrings[0]));

        // Return an empty command group if no auto is specified
        // return new SequentialCommandGroup();
    }

    public Command getAutonomousCommand() {
        return loadAutonomousCommand();
    }
}
