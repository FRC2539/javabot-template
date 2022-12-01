package frc.lib.logging;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;

public class LoggableDoubleArray {
    DoubleArrayPublisher posePublisher;
    DoubleArrayLogEntry poseLogger;
    double[] poseArray;

    public LoggableDoubleArray(DoubleArrayPublisher posePublisher, DoubleArrayLogEntry poseLogger, double[] poseArray) {
        this.posePublisher = posePublisher;
        this.poseLogger = poseLogger;
        this.poseArray = poseArray;
    }

    public void logAndPublishPoses(double[] poseArray) {
        posePublisher.set(poseArray);
        poseLogger.append(poseArray);
    }
}
