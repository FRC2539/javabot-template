package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.ArrayList;
import java.util.HashMap;

public class FollowTrajectoryGroupCommand extends CommandBase {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private ArrayList<PathPlannerTrajectory> trajectories;
    private ArrayList<Command> simultaneousCommands;
    private ArrayList<Command> stopPointCommands;
    private HashMap<String, Command> markerCommands;
    private ArrayList<EventMarker> unpassedMarkers;
    private Command simultaneousCommand;
    private Command stopPointCommand;
    private Timer timer;
    private State state;

    private enum State {
        MovingExecute,
        MovingInitialize,
        StoppedExecute,
        StoppedInitialize,
        Finished
    }

    public FollowTrajectoryGroupCommand(
            SwerveDriveSubsystem swerveDriveSubsystem,
            ArrayList<PathPlannerTrajectory> trajectories,
            ArrayList<Command> simultaneousCommands,
            ArrayList<Command> stopPointCommands,
            HashMap<String, Command> markerCommands) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.trajectories = trajectories;
        this.simultaneousCommands = simultaneousCommands;
        this.stopPointCommands = stopPointCommands;
        this.markerCommands = markerCommands;

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        state = State.StoppedInitialize;
        simultaneousCommand = new InstantCommand();
        stopPointCommand = new InstantCommand();
    }

    @Override
    public void execute() {
        switch (state) {
            case StoppedInitialize:
                // checks if there are any stop point commmands left to execute
                if (stopPointCommands.size() > 0) {
                    // if so, schedules it and goes to stopped exe
                    stopPointCommand = stopPointCommands.remove(0);
                    stopPointCommand.schedule();
                    state = State.StoppedExecute;
                } else {
                    // else skips to the path following phase
                    state = State.MovingInitialize;
                }
                break;
            case StoppedExecute:
                // if the stop point command is finished, it goes to the path following phase
                if (stopPointCommand.isFinished()) {
                    state = State.MovingInitialize;
                }
                break;
            case MovingInitialize:
                unpassedMarkers = new ArrayList<>();
                // checks if there are any remaining trajectories to follow
                if (trajectories.size() > 0) {
                    // if so, records all of the markers
                    unpassedMarkers.addAll(trajectories.get(0).getMarkers());

                    // follows the trajectory
                    swerveDriveSubsystem.getFollower().follow(trajectories.remove(0));

                    // if there are remaining simultaneous commands to go with the trajectories, it schedules them with
                    // it, otherwise it just substitutes
                    // in an instant command and goes to the MovingExecute
                    if (simultaneousCommands.size() > 0) simultaneousCommand = simultaneousCommands.remove(0);
                    else simultaneousCommand = new InstantCommand();

                    simultaneousCommand.schedule();
                    timer.reset();

                    state = State.MovingExecute;
                } else {
                    // else, finishes cuz there is nothing left to do.
                    state = State.Finished;
                }
                break;
            case MovingExecute:
                // if a marker has had it's time pass, it runs the command associated with it
                if (unpassedMarkers.size() > 0 && timer.get() >= unpassedMarkers.get(0).timeSeconds) {
                    markerCommands.get(unpassedMarkers.remove(0).name).schedule();
                }

                // if the trajectory is finished being followed, it stops following the trajectory, ends the
                // simultaneousCommand, and sets the state to start stopping
                if (swerveDriveSubsystem.getFollower().getCurrentTrajectory().isEmpty()) {
                    simultaneousCommand.end(true);
                    swerveDriveSubsystem.getFollower().cancel();
                    state = State.StoppedInitialize;
                }
                break;
            case Finished:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.Finished;
    }
}
