package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.TimesliceConstants;

public class Robot extends TimesliceRobot {
    public static CTREConfigs ctreConfigs = new CTREConfigs();

    private RobotContainer robotContainer;

    public Robot() {
        super(TimesliceConstants.ROBOT_PERIODIC_ALLOCATION, TimesliceConstants.CONTROLLER_PERIOD);

        robotContainer = new RobotContainer(this);

        // Prevents the logging of many errors with our controllers
        DriverStation.silenceJoystickConnectionWarning(true);

        // Begin logging networktables, controller inputs, and more
        if (isReal()) {
            DriverStation.startDataLog(DataLogManager.getLog()); // getLog() automatically starts the logging
        }
    }

    @Override
    public void robotInit() {}

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        robotContainer.getAutonomousCommand().schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}
