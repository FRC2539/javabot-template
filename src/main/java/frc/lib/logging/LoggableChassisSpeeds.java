package frc.lib.logging;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class LoggableChassisSpeeds {
    DoubleArrayTopic topic;
    DoubleArrayPublisher publisher;
    DoubleArrayLogEntry logger;
    LoggableDoubleArray defaultValue;
    boolean override = !Constants.competitionMode;

    public LoggableChassisSpeeds(String path, ChassisSpeeds defaultValue) {
        this.defaultValue = new LoggableDoubleArray(path, toDoubleArray(defaultValue));

        NetworkTableInstance.getDefault().getDoubleArrayTopic(path).publish();
        logger = new DoubleArrayLogEntry(DataLogManager.getLog(), path);
    }

    public LoggableChassisSpeeds(String path, ChassisSpeeds defaultValue, boolean override) {
        this(path, defaultValue);
        this.override = override;
    }

    public void set(ChassisSpeeds value) {
        
        if (override) publisher.set(toDoubleArray(value));

        logger.append(toDoubleArray(value));
    }

    private double[] toDoubleArray(ChassisSpeeds value) {
        var doubleArray = new double[] {value.vxMetersPerSecond, value.vyMetersPerSecond, value.omegaRadiansPerSecond};
        return doubleArray;
    }
    
}
