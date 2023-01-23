package frc.robot.ShamLib.SMF;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.*;
import java.util.Map.Entry;

public class SubsystemManager {
    private static SubsystemManager instance;
    private List<StatedSubsystem<?>> subsystems = new ArrayList<>();

    SubsystemManager() {}

    public static synchronized SubsystemManager getInstance() {
        if(instance == null) {
            instance = new SubsystemManager();
        }
        return instance;
    }

    /**
     * Add a subsystem to be tracked by the SubsystemManager instance. It will automatically enable and disable it.
     * @param subsystem subsystem to add to the manager
     */
    public void registerSubsystem(StatedSubsystem<?> subsystem) {

        if(!subsystems.contains(subsystem)) {
            sendSubsystemToNT(subsystem);

            subsystems.add(subsystem);
        } else {
            System.out.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
            System.out.println("YOU'VE ATTEMPTED TO REGISTER A SUBSYSTEM THAT YOU'VE ALREADY REGISTERED");
            System.out.println("Subsystem info: " + subsystem.getName());
            System.out.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
        }

    }

    /**
     * Send a subsystem through networktables under SmartDashboard
     * @param subsystem the subsystem to be sent to network tables
     */
    public void sendSubsystemToNT(StatedSubsystem<?> subsystem) {
        //Send the subsystem itself to network tables
        SmartDashboard.putData(subsystem.getName(), subsystem);

        //Send any additional sendables that should be included in the subsystem
        for(Map.Entry<String, Sendable> e : subsystem.additionalSendables().entrySet()) {
            SmartDashboard.putData(subsystem.getName() + "/" + e.getKey(), e.getValue());
        }
    }


    /**
     * Register a number of subsystems at once
     * @param subsystems array of stated subsystems 
     */
    public void registerSubsystems(StatedSubsystem<?> ...subsystems) {
        for(StatedSubsystem<?> s : subsystems) {
            registerSubsystem(s);
        }
    }

    /**
     * Compose a parallel command group  that will determine the state of every subsystem that has not yet determined itself
     * @return the command group that will determined every subsystem
     */
    public Command determineAllSubsystems() {
        List<Command> commands = new ArrayList<>();

        for(StatedSubsystem s : subsystems) {
            if(s.isUndetermined()) {  
                commands.add(s.goToStateCommand(s.getEntryState()));
            }
        }
        return new ParallelCommandGroup(commands.toArray(new Command[commands.size()]));
    }

    /**
     * Tell all subsystems that they should be enabled
     */
    public void enableAllSubsystems() {
        for(StatedSubsystem s : subsystems) {
            s.enable();
        }
    }

    /**
     * Tell all subsystems that they should be disabled
     */
    public void disableAllSubsystems() {
        for(StatedSubsystem s : subsystems) {
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
