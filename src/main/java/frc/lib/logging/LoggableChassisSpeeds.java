package frc.lib.logging;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class LoggableChassisSpeeds {
    DoubleArrayPublisher publisher;
    DoubleArrayLogEntry logger;
    LoggableDoubleArray defaultValue;
    boolean override = !Constants.competitionMode;

    public LoggableChassisSpeeds(String path, ChassisSpeeds defaultValue) {
        this.defaultValue = new LoggableDoubleArray(path, toDoubleArray(defaultValue));

        publisher = NetworkTableInstance.getDefault().getDoubleArrayTopic(path).publish();
        logger = new DoubleArrayLogEntry(DataLogManager.getLog(), path);
    }

    public LoggableChassisSpeeds(String path, ChassisSpeeds defaultValue, boolean override) {
        this(path, defaultValue);
        this.override = override;
    }

    public void set(ChassisSpeeds value) {
        var valueAsArray = toDoubleArray(value);

        if (override) publisher.set(valueAsArray);

        logger.append(valueAsArray);
    }

    private double[] toDoubleArray(ChassisSpeeds value) {
        var doubleArray = new double[] {value.vxMetersPerSecond, value.vyMetersPerSecond, value.omegaRadiansPerSecond};
        return doubleArray;
    }
    
}
