package frc.lib.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class LoggableString {
    StringTopic topic;
    StringPublisher publisher;
    StringSubscriber subscriber;
    StringLogEntry logger;
    String defaultValue;

    /**
     * @param path The full name of the double, e.g. "/MySubsystem/MyThing"
     * @param defaultValue
     */
    public LoggableString(String path, String defaultValue) {
        this.defaultValue = defaultValue;

        topic = NetworkTableInstance.getDefault().getStringTopic(path);
        logger = new StringLogEntry(DataLogManager.getLog(), path);
    }

    public void set(String value) {
        // Lazily create a publisher
        if (publisher == null) publisher = topic.publish();

        publisher.set(value);
        logger.append(value);
    }

    public String get() {
        // Lazily create a subscriber
        if (subscriber == null) subscriber = topic.subscribe(defaultValue);

        var value = subscriber.get();
        logger.append(value);

        return value;
    }
}
