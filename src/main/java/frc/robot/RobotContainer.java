package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.controller.Axis;
import frc.lib.controller.ThrustmasterJoystick;
import frc.lib.loops.UpdateManager;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TimesliceConstants;
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
    private UpdateManager updateManager;

    public RobotContainer(TimesliceRobot robot) {
        trajectoryLoader = new TrajectoryLoader();
        updateManager = new UpdateManager(robot);
        autonomousManager = new AutonomousManager(trajectoryLoader, this);

        updateManager.schedule(swerveDriveSubsystem, TimesliceConstants.DRIVETRAIN_PERIOD);

        swerveDriveSubsystem.setDefaultCommand(
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

        rightDriveController.sendButtonNamesToNT();
        leftDriveController.sendButtonNamesToNT();
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
