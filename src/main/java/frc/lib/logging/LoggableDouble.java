package frc.lib.logging;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class LoggableDouble {
    DoubleTopic topic;
    DoublePublisher publisher;
    DoubleSubscriber subscriber;
    DoubleLogEntry logger;
    double defaultValue;

    /**
     * @param path The full name of the double, e.g. "/MySubsystem/MyThing"
     * @param initialArray
     */
    public LoggableDouble(String path, double defaultValue) {
        this.defaultValue = defaultValue;

        topic = NetworkTableInstance.getDefault().getDoubleTopic(path);
        logger = new DoubleLogEntry(DataLogManager.getLog(), path);
    }

    public void set(double value) {
        // Lazily create a publisher
        if (publisher == null) publisher = topic.publish();

        publisher.set(value);
        logger.append(value);
    }

    public double get() {
        // Lazily create a subscriber
        if (subscriber == null) subscriber = topic.subscribe(defaultValue);

        var value = subscriber.get();
        logger.append(value);

        return value;
    }
}
