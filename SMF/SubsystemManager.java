package frc.robot.ShamLib.SMF;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.*;

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
            sendOnNt(subsystem);
        }
    }

    private void sendOnNt(StateMachine<?> subsystem) {
        SmartDashboard.putData(subsystem.getName(), subsystem);

        for(Map.Entry<String, Sendable> entry : subsystem.additionalSendables().entrySet()) {
            SmartDashboard.putData("/" + subsystem.getName() + "/" + entry.getKey(), entry.getValue());
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
     * @return the command group that will determine every subsystem
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
     * Enable all subsystems and determine all subsystems
     */
    public void prepSubsystems() {
        enableAllSubsystems();
        determineAllSubsystems();
    }

}
