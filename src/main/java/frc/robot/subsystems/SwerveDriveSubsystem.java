package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.control.MovingAverageVelocity;
import frc.lib.control.SwerveDriveSignal;
import frc.lib.loops.Updatable;
import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.TimesliceConstants;
import frc.robot.util.TrajectoryFollower;

/**
 * SwerveDriveSubsystem
 */
public class SwerveDriveSubsystem extends SubsystemBase implements Updatable {
    public final PIDController autoXController = new PIDController(1, 0, 0, TimesliceConstants.CONTROLLER_PERIOD);
    public final PIDController autoYController = new PIDController(1, 0, 0, TimesliceConstants.CONTROLLER_PERIOD);
    public final ProfiledPIDController autoThetaController = new ProfiledPIDController(
            0.17,
            0,
            0.07,
            new TrapezoidProfile.Constraints(
                    Constants.SwerveConstants.maxSpeed, Constants.SwerveConstants.maxAngularVelocity),
            TimesliceConstants.CONTROLLER_PERIOD);

    private final TrajectoryFollower follower =
            new TrajectoryFollower(autoXController, autoYController, autoThetaController);

    // private final SwerveDrivePoseEstimator<N7, N7, N5> swervePoseEstimator;
    private final SwerveDriveOdometry swervePoseEstimator;

    private final MovingAverageVelocity velocityEstimator = new MovingAverageVelocity(20);

    private Pose2d pose = new Pose2d();
    private ChassisSpeeds velocity = new ChassisSpeeds();
    private SwerveDriveSignal driveSignal = new SwerveDriveSignal();

    private SwerveModule[] modules;

    private final Pigeon2 gyro = new Pigeon2(60);

    private NetworkTable table;
    private DoubleArrayPublisher posePublisher;
    private DoubleArrayLogEntry poseLogger = new DoubleArrayLogEntry(DataLogManager.getLog(), "Pose");

    private DoubleArrayPublisher cancoderPublisher;

    private DoublePublisher doublePublisher;

    //NOTE: This is a workaround because Pathplanner has not yet released their newest build of their thingy.
    //Should be changed eventually and the code relating to it should be updated.
    // public boolean isTakingModuleStatesNotChassisSpeeds = false;

    public SwerveDriveSubsystem() {
        table = NetworkTableInstance.getDefault().getTable(getName());
        posePublisher = table.getDoubleArrayTopic("Pose").publish();

        cancoderPublisher = table.getDoubleArrayTopic("CANCoders").publish();

        doublePublisher = table.getDoubleTopic("Double").publish();

        modules = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        // Reset each module using its absolute encoder to avoid having modules fail to align
        for (SwerveModule module : modules) {
            module.resetToAbsolute();
        }

        // Initialize the swerve drive pose estimator with access to the module positions.
        // swervePoseEstimator = new SwerveDrivePoseEstimator<N7, N7, N5>(
        //         Nat.N7(),
        //         Nat.N7(),
        //         Nat.N5(),
        //         new Rotation2d(),
        //         getModulePositions(),
        //         new Pose2d(),
        //         Constants.SwerveConstants.swerveKinematics,
        //         VecBuilder.fill(0.001, 0.001, Units.degreesToRadians(0.001), 0.001, 0.001, 0.001, 0.001),
        //         VecBuilder.fill(Units.degreesToRadians(0.001), 0.001, 0.001, 0.001, 0.001),
        //         VecBuilder.fill(0.025, 0.025, Units.degreesToRadians(0.025)));

        swervePoseEstimator = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, new Rotation2d(), getModulePositions(), pose);

        // Flip the initial pose estimate to match the practice pose estimate to the post-auto pose estimate
        // setRotation(Rotation2d.fromDegrees(180));
    }

    // public SwerveDrivePoseEstimator<N7, N7, N5> getPoseEstimator() {
    //     return swervePoseEstimator;
    // }

    public Pose2d getPose() {
        return pose;
    }

    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    public ChassisSpeeds getSmoothedVelocity() {
        return velocityEstimator.getAverage();
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getRotation() {
        return pose.getRotation();
    }

    public void drive(ChassisSpeeds velocity) {
        drive(velocity, false);
    }

    public void drive(ChassisSpeeds velocity, boolean isFieldOriented) {
        driveSignal = new SwerveDriveSignal(velocity, isFieldOriented);
    }

    public void stop() {
        driveSignal = new SwerveDriveSignal();
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
        swervePoseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
    }

    public void setRotation(Rotation2d angle) {
        setPose(new Pose2d(getPose().getX(), getPose().getY(), angle));
    }

    public void zeroRotation() {
        setRotation(new Rotation2d());
    }

    public double[] getDriveTemperatures() {
        return new double[] {
            modules[0].getDriveTemperature(),
            modules[1].getDriveTemperature(),
            modules[2].getDriveTemperature(),
            modules[3].getDriveTemperature()
        };
    }

    public double[] getSteerTemperatures() {
        return new double[] {
            modules[0].getSteerTemperature(),
            modules[1].getSteerTemperature(),
            modules[2].getSteerTemperature(),
            modules[3].getSteerTemperature()
        };
    }

    private void updateOdometry() {
        SwerveModuleState[] moduleStates = getModuleStates();
        SwerveModulePosition[] modulePositions = getModulePositions();

        velocity = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(moduleStates);

        velocityEstimator.add(velocity);

        // pose = swervePoseEstimator.updateWithTime(
        //         Timer.getFPGATimestamp(), gyro.getRotation2d(), moduleStates, modulePositions);

        pose = swervePoseEstimator.update(getGyroRotation(), modulePositions);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule module : modules) {
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }

    private void updateModules(SwerveDriveSignal driveSignal) {
        ChassisSpeeds chassisVelocity;

        if (driveSignal.isFieldOriented()) {
            chassisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveSignal.vxMetersPerSecond,
                    driveSignal.vyMetersPerSecond,
                    driveSignal.omegaRadiansPerSecond,
                    getRotation());
        } else {
            chassisVelocity = new ChassisSpeeds(
                    driveSignal.vxMetersPerSecond, driveSignal.vyMetersPerSecond, driveSignal.omegaRadiansPerSecond);
        }

        SwerveModuleState[] moduleStates =
                Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisVelocity);

        setModuleStates(moduleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModule module : modules) {
            module.setDesiredState(desiredStates[module.moduleNumber], true);
        }
    }

    public void setModuleStatesProxy(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

        driveSignal = new SwerveDriveSignal(Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(desiredStates), false);
    }

    @Override
    public void update() {
        updateOdometry();

        updateModules(driveSignal);
    }

    @Override
    public void periodic() {
        // double[] poseArray = poseToDoubleArray(pose);

        // LoggableDoubleArray loggablePoseArray = new LoggableDoubleArray(posePublisher, poseLogger, poseArray);
        // loggablePoseArray.logAndPublishPoses(poseArray);

        doublePublisher.set(pose.getRotation().getDegrees());
    }

    private double[] poseToDoubleArray(Pose2d pose) {
        return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
    }

    public TrajectoryFollower getFollower() {
        return follower;
    }
}
