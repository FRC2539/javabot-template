package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.controller.Axis;
import frc.lib.controller.ThrustmasterJoystick;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.AutonomousManager;
import frc.robot.util.TrajectoryLoader;

public class RobotContainer {
    private final ThrustmasterJoystick leftDriveController =
            new ThrustmasterJoystick(ControllerConstants.LEFT_DRIVE_CONTROLLER);
    private final ThrustmasterJoystick rightDriveController =
            new ThrustmasterJoystick(ControllerConstants.RIGHT_DRIVE_CONTROLLER);

    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();

    private AutonomousManager autonomousManager;
    private TrajectoryLoader trajectoryLoader;

    public RobotContainer() {
        trajectoryLoader = new TrajectoryLoader();

        autonomousManager = new AutonomousManager(trajectoryLoader, this);

        CommandScheduler.getInstance().registerSubsystem(swerveDriveSubsystem);

        CommandScheduler.getInstance()
                .setDefaultCommand(
                        swerveDriveSubsystem,
                        new DriveCommand(
                                swerveDriveSubsystem,
                                getDriveForwardAxis(),
                                getDriveStrafeAxis(),
                                getDriveRotationAxis()));

        configureControllerLayout();
    }

    private void configureControllerLayout() {
        leftDriveController.getXAxis().setScale(Constants.SwerveConstants.maxSpeed);
        leftDriveController.getYAxis().setScale(Constants.SwerveConstants.maxSpeed);
        rightDriveController.getXAxis().setScale(Constants.SwerveConstants.maxAngularVelocity);

        leftDriveController.getLeftTopLeft().onTrue(new InstantCommand(() -> swerveDriveSubsystem.resetGyroAngle()));
        leftDriveController
                .getLeftTopRight()
                .onTrue(new InstantCommand(() -> swerveDriveSubsystem.resetPose(new Pose2d(
                        new Translation2d(),
                        swerveDriveSubsystem.getGyroRotation2d().rotateBy(Rotation2d.fromDegrees(180))))));
    }

    public Command getAutonomousCommand() {
        return autonomousManager.getAutonomousCommand();
    }

    public Axis getDriveForwardAxis() {
        return leftDriveController.getYAxis();
    }

    public Axis getDriveStrafeAxis() {
        return leftDriveController.getXAxis();
    }

    public Axis getDriveRotationAxis() {
        return rightDriveController.getXAxis();
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return swerveDriveSubsystem;
    }
}
