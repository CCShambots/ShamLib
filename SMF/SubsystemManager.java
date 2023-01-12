package frc.robot.ShamLib.SMF;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.*;
import java.util.Map.Entry;

public class SubsystemManager {
    private final List<StateMachine<?>> subsystems = new ArrayList<>();

    SubsystemManager() {}


    /**
     * Add a subsystem to be tracked by the SubsystemManager instance. It will automatically enable and disable it.
     * @param subsystem subsystem to add to the manager
     */
    public void registerSubsystem(StateMachine<?> subsystem) {
        if(!subsystems.contains(subsystem)) {
            subsystems.add(subsystem);
        }
    }

    /**
     * Register a number of subsystems at once
     * @param subsystems array of stated subsystems 
     */
    public void registerSubsystems(StateMachine<?> ...subsystems) {
        for(StateMachine<?> s : subsystems) {
            registerSubsystem(s);
        }
    }

    /**
     * Compose a parallel command group  that will determine the state of every subsystem that has not yet determined itself
     * @return the command group that will determined every subsystem
     */
    public void determineAllSubsystems() {
        for (StateMachine<?> sm : subsystems) {
            sm.determineState();
        }
    }

    /**
     * Tell all subsystems that they should be enabled
     */
    public void enableAllSubsystems() {
        for(StateMachine s : subsystems) {
            s.enable();
        }
    }

    /**
     * Tell all subsystems that they should be disabled
     */
    public void disableAllSubsystems() {
        for(StateMachine s : subsystems) {
            s.disable();
        }
    }

    /**
     * Enable all subsystems and return the command to determine all subsystems
     * @return the command that should be run
     */
    public Command prepSubsystems() {
        enableAllSubsystems();
        return determineAllSubsystems();
    }

}
