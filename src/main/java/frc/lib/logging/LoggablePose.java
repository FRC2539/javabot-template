package frc.lib.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;

public class LoggablePose {
    DoubleArrayPublisher publisher;
    DoubleArrayLogEntry logger;
    LoggableDoubleArray defaultValue;
    boolean override = !Constants.competitionMode;

    public LoggablePose(String path, Pose2d defaultValue) {
        this.defaultValue = new LoggableDoubleArray(path, toDoubleArray(defaultValue));

        publisher = NetworkTableInstance.getDefault().getDoubleArrayTopic(path).publish();
        logger = new DoubleArrayLogEntry(DataLogManager.getLog(), path);
    }

    public LoggablePose(String path, Pose2d defaultValue, boolean override) {
        this(path, defaultValue);
        this.override = override;
    }

    public void set(Pose2d value) {
        var valueAsArray = toDoubleArray(value);

        if (override) publisher.set(valueAsArray);

        logger.append(valueAsArray);
    }

    private double[] toDoubleArray(Pose2d pose) {
        var doubleArray =
                new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
        return doubleArray;
    }
}
