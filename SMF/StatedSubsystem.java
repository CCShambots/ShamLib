package frc.robot.ShamLib.SMF;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ShamLib.SMF.exceptions.DuplicateTransitionException;
import frc.robot.ShamLib.SMF.exceptions.InstanceBasedStateException;
import frc.robot.ShamLib.SMF.exceptions.InvalidContinuousCommandException;
import frc.robot.ShamLib.SMF.exceptions.InvalidTransitionException;
import frc.robot.ShamLib.SMF.exceptions.InstanceBasedStateException.InstanceBasedReason;
import frc.robot.ShamLib.SMF.exceptions.InvalidContinuousCommandException.ContinuousCommandReason;
import frc.robot.ShamLib.SMF.exceptions.InvalidTransitionException.TransitionReason;

import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

public abstract class StatedSubsystem<E extends Enum<E>> extends SubsystemBase {

    private final List<Transition<E>> transitions = new ArrayList<>();
    private final Map<E, List<FlagState<E>>> flagStates;
    private final List<E> perInstanceStates = new ArrayList<>();
    private final Map<E, Command> continuousCommands;
    private E undeterminedState;
    private E entryState;

    private E currentState;
    private E flagState;
    private E desiredState;

    private Transition<E> currentTransition;
    private boolean transitioning = false;
    private Command currentCommand = null;
    private boolean needToScheduleTransitionCommand = false;
    private boolean enabled = false;
    
    public StatedSubsystem(Class<E> enumType) {
        flagStates = new EnumMap<>(enumType);
        continuousCommands = new EnumMap<>(enumType);
    }

    /**
     * Alternative version to define states that can transition between each other
     * @param state1 One state
     * @param state2 The other state
     * @param command1 The command that will run from state1 to state2
     * @param command2 The command that will run from state2 to state1
     */
    protected void addCommutativeTransition(E state1, E state2, Command command1, Command command2) {
        addTransition(state1, state2, command1);
        addTransition(state2, state1, command2);
    }

    /**
     * Creates a new transition with a duplicate command as another one. This is useful for running the same transition command between / among many states
     * @param startState the start state of the transition you want to create
     * @param endState the ending state of the transition you want to create
     * @param copyFromStart the starting state of the transition you want to copy
     * @param copyFromEnd the ending state of the transition you want to copy
     * @return whether a transition was successfully found
     */
    protected boolean duplicateTransition(E startState, E endState, E copyFromStart, E copyFromEnd) {

        for(Transition<E> t : transitions) {
            if(t.getStartState() == copyFromStart && t.getEndState() == copyFromEnd) {
                addTransition(startState, endState, t.getCommand());
                return true;
            }
        }

        return false;
    }

    /**
     * Create a transition between / among many states easily. A transition with the same command will be created going from every provided start state
     * to any provided end state
     * @param startingStates all starting states to apply the transitions to
     * @param endingStates all ending states to apply the transitions to
     * @param command the command to run
     */
    protected void addOmniTransition(E[] startingStates, E[] endingStates, Command command) {
        for(E start : startingStates) {
            for (E end : endingStates) {
                addTransition(start, end, command);
            }
        }
    }

    /**
     * Create transitions to one end state from many start states.
     * This can be useful for cancelling subroutines that consist of many states
     * @param endState the one state that you are going towards
     * @param command The command to be run for the transition
     * @param startStates the states which can go to the end state
     */
    protected void addMultiTransitions(E endState, Command command, E... startStates) {
       for(E state : startStates) {
           addTransition(state, endState, command);
       }
    }

    /**
     * Default version of this method reverts to the start state in case of an interruption
     * @param startState where the subsystem is when beginning the transition
     * @param endState where the subsystem ends the transition
     * @param command the command that runs
     */
    protected void addTransition(E  startState, E endState, Command command) {
        addTransition(startState, endState, startState, command);
    }

    /**
     * Alternate version of the method where an interruption state can be stated
     * @param startState where the subsystem is when beginning the transition
     * @param endState where the subsystem ends the transition
     * @param interruptionState the state that should be switched to if the transition is interrupted
     * @param command the command that runs through the transition
     */
    protected void addTransition(E startState, E endState, E interruptionState, Command command) {
        Transition<E> suggestedTransition = new Transition<>(startState, endState, interruptionState, command);

        //End the function if the transition conflicts
        if(!checkIfValidTransition(suggestedTransition)) return;

        transitions.add(suggestedTransition);
    }

    /**
     * Give the command that allows the
     * @param undeterminedState The state in which the subsystem is initialized
     * @param entryState The state the subsystem can go to from the undetermined state
     * @param command The command that runs in the transition
     */
    protected void addDetermination(E undeterminedState, E entryState, Command command) {
        if(this.undeterminedState != null) {
            outputErrorMessage("You've already defined an undetermined state for this subsystem",
                    "(Subsystem name: " + getName() + ")");
            return;
        }

        Transition<E> determinationTransition = new Transition<>(undeterminedState, entryState, command);

        if(!checkIfValidTransition(determinationTransition)) return;

        this.undeterminedState = undeterminedState;
        this.entryState = entryState;
        this.currentState = undeterminedState;
        this.desiredState = undeterminedState;

        transitions.add(determinationTransition);
    }

    /**
     * Add a new flag state to better indicate the subsystem's state
     * If a subsystem fulfills two flag states at once, the one it displays will be unreliable
     * NOTE: If you add two of the same flag state, unexpected behavior will occur
     * @param parentState the parent state in which the flag state should trigger
     * @param flagState the flag state which can potentially be active
     * @param condition under what conditions the flag state should be active
     */
    protected void addFlagState(E parentState, E flagState, BooleanSupplier condition) {

        //If the flag state is already registered as part of transitions, it is invalid
        if(getStartStates().contains(flagState) || getEndStates().contains(flagState) || getInterruptionStates().contains(flagState)) {
            outputErrorMessage("FLAG STATES CANNOT BE PARTS OF TRANSITIONS",
                    "You tried to register a flag state that is already the start, end, or interruption state or an existing transition",
                    "Parent state: " + parentState.name(),
                    "Flag state: " + flagState.name()
                    );
            return;

        }

        //Any state marked as a flag state cannot be a parent state
        if(getStatesMarkedAsFlag().contains(parentState)) {
            outputErrorMessage("FLAG STATES CANNOT BE USED AS PARENT STATES",
                    "You attempted to register the following:",
                    "Parent state: " + parentState.name() + " (this is already a flag state)",
                    "Flag state: " + flagState.name());
            return;
        }

        //Any state already marked as a parent state cannot become a flag state
        if(getStatesMarkedAsParent().contains(flagState)) {

            outputErrorMessage("PARENTS STATES CANNOT BE MADE FLAG STATES",
                    "You attempted to register a flag state that has already been marked a parent state",
                    "Parent state: " + parentState.name(),
                    "Flag state: " + flagState.name() + " (already a parent state)");

            return;
        }

        //Register the parent state as a new parent state if it does not yet have flag states
        if(!flagStates.containsKey(parentState)) flagStates.put(parentState, new ArrayList<>());

        flagStates.get(parentState).add(new FlagState<>(flagState, condition));
    }

    /**
     * Set a command to schedule once a state is reached.
     * This command should run indefinitely (i.e. isFinished() should never return true).
     * The command will be canceled when a transition begins.
     * Only the first command registered for a certain state will be
     * @param state the state that should have a continuous command
     * @param command the command that should run once the subsystem reaches that state
     * @return whether the command was successfully added
     */
    protected boolean setContinuousCommand(E state, Command command) {
        try {
            if(!isValidCommand(command)) {
                throw new InvalidContinuousCommandException(getName(), command, state.name(), ContinuousCommandReason.InvalidCommand);
            }
    
            //The continuous command is invalid if the indicated state is a flag state
            if(isFlagState(state))
                throw new InvalidContinuousCommandException(getName(), command, state.name(), ContinuousCommandReason.AlreadyFlagState);
            if(continuousCommands.containsKey(state)){
                throw new InvalidContinuousCommandException(getName(), command, state.name(), ContinuousCommandReason.CommandAlreadyDefined);
            } else if(perInstanceStates.contains(state)) {
                throw new InvalidContinuousCommandException(getName(), command, state.name(), ContinuousCommandReason.InstanceBasedState);
            }
            else continuousCommands.put(state, command);

            return true;
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    /**
     * Define a state that will start a different continuous command each time it is reached
     * This state cannot be a flag state, or a state with a continuous command
     * @param state the state that should become instance-based
     * @return whether the state was successfully marked as instance-based
     */
    protected boolean addInstanceBasedState(E state) {
        try {
            if(isFlagState(state)) throw new InstanceBasedStateException(getName(), state.name(), InstanceBasedReason.AlreadyFlagState);
    
            if(!continuousCommands.containsKey(state)) perInstanceStates.add(state);
            else throw new InstanceBasedStateException(getName(), state.name(), InstanceBasedReason.AlreadyContinuous);

            return true;
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    private boolean isFlagState(E state) {
        Set<E> statesMarkedAsFlag = getStatesMarkedAsFlag();
        if(statesMarkedAsFlag.contains(state)) {

            return true;
        }
        return statesMarkedAsFlag.contains(state);
    }

    /**
     * Determined whether a given command is valid.
     * Commands must either require ONLY the subsystem for which they are being used, OR have no requirements
     * @param command the proposed command
     * @return if the proposed command is valid or not
     */
    private boolean isValidCommand(Command command) {
        //Commands should only require this one subsystem or no subsystem at all
        if(command.getRequirements().size() > 1) return false;
        else return command.getRequirements().contains(this) || command.getRequirements().size() != 1;
    }

    /**
     * @param proposedTransition The transition that should be compared against all the other transitions
     * @return whether the proposed transition is valid or not
     */
    private boolean checkIfValidTransition(Transition<E> proposedTransition) {
        try {
            //The transition is invalid if it leads from or to the undetermined state
            if(this.undeterminedState == proposedTransition.getStartState() || this.undeterminedState == proposedTransition.getEndState()) {
                outputErrorMessage("TRANSITIONS CANNOT GO TO OR FROM THE UNDETERMINED STATE", "Transition information: " + proposedTransition);

                return false;
            }

            //The transition is invalid if the command itself is invalid (has incorrect subsystem requirements)
            if(!isValidCommand(proposedTransition.getCommand())) {
                throw new InvalidTransitionException(getName(), proposedTransition, TransitionReason.CommandRequirements);
            }

            //The transition is invalid if either one of its states have been marked as flag states
            Set<E> statesMarkedAsFlag = getStatesMarkedAsFlag();
            if(statesMarkedAsFlag.contains(proposedTransition.getStartState()) || statesMarkedAsFlag.contains(proposedTransition.getEndState()) || statesMarkedAsFlag.contains(proposedTransition.getInterruptionState())) {
                throw new InvalidTransitionException(getName(), proposedTransition, TransitionReason.StateAlreadyFlag);
            }

            //Check the other defined transitions to see if there are any conflicts (if they represent the same states, or if they
            for(Transition<E> t : transitions) {
                if(!proposedTransition.isValidTransition(t)) {
                    throw new DuplicateTransitionException(getName(), proposedTransition, t);
                }
            }

            return true;
        } catch (Exception e) {
            e.printStackTrace();

            return false;
        }
        
    }

    /**
     * Method for controlling the state of the subsystem. DO NOT OVERRIDE this as you would in a normal subsystem
     */
    @Override
    public final void periodic() {

        //States can only be managed whilst the subsystem is enabled
        if(enabled) {

            //If a command needs to be scheduled, and the previous command that was canceled is no longer scheduled, schedule the transition command
            if(needToScheduleTransitionCommand) {
                needToScheduleTransitionCommand = false;
                currentCommand = currentTransition.getCommand();
                currentCommand.schedule();
            }

            //If a transition has finished, check for a continuous command
            if(currentCommand != null) {
                if(transitioning && !currentCommand.isScheduled()) {
                    transitioning = false;
                    currentState = desiredState;
                    flagState = null;
                    currentCommand = continuousCommands.get(currentState);
    
                    //Add a decorator to the command that will remove the instance-based
                    //command once it has finished running
                    if(perInstanceStates.contains(currentState)) {
                        continuousCommands.remove(currentState);
                    }
                    
                    if(currentCommand != null) {
                        currentCommand.schedule();
                    }
                }
            }

            //Check for flag states and activate exactly one of them
            //There is only one flag position potentially active at a time
            //Also negate the existing flag state if its condition is no longer true
            if(flagStates.get(currentState) != null) {
                for(FlagState<E> f : flagStates.get(currentState)) {
                    if(f.getCondition().getAsBoolean()) {
                        flagState = f.getState();
                    } else if(f.getState() == flagState) {
                        flagState = null;
                    }
                }
            }
        }

        update();
    }

    /**
     * Method that acts as a replacement for periodic()
     * DO NOT Override periodic() in your StatedSubsystems
     */
    public abstract void update();


    //TODO: Actual error handling instead of just prints
    /**
     * Ask for the subsystem to move to a different state
     * If a flag state is provided, the robot will start transitioning to its parent state
     * @param state The state to which the subsystem should go
     * @return whether a transition was successfully found
     */
    public boolean requestTransition(E state) {

        //Since this sate has been marked as a flag state, we find its parent state and request to transition to that
        if(getStatesMarkedAsFlag().contains(state)) {
            state = findParentState(state);
        }

        if(!getEndStates().contains(state)) {
            outputErrorMessage("YOU TRIED TO REQUEST A STATE THAT DOESN'T EXIST",
                    "SUBSYSTEM NAME: " + getName(),
                    "TRANSITION CONFLICT: " + state.name());
            return false;
        }

        Transition<E> t = findTransition(currentState, state, transitions);

        //If a valid transition was found, then start performing it
        if(t != null) {

            //If a transition is already occurring, cancel it
            if(transitioning) {
                cancelTransition();
            }

            //If a continuous command is running, cancel it
            if(!transitioning && currentCommand != null) {
                currentCommand.cancel();
            }

            desiredState = state;
            currentTransition = t;
            needToScheduleTransitionCommand = true;
            transitioning = true;
        } else return false;

        return true;
    }

    /**
     * Request that a subsystem move to a target state (for instance-based commands)
     * @param state the desired state the subsystem should go to
     * @param command the command that should be run for this instance
     * @return whether a transition was successfully found
     */
    public boolean requestTransition(E state, Command command) {
        if(perInstanceStates.contains(state)) {
            continuousCommands.put(state, command);
        } else return false;

        return requestTransition(state);
    }

    /**
     * Search the list of transitions to find which transition should be implemented
     * @return Whatever transition is found. This can be null
     */
    private Transition<E> findTransition(E startState, E endState, List<Transition<E>> transitions) {
        for(Transition<E> t : transitions) {
            if(startState==t.getStartState() && endState==t.getEndState()) return t;
        }

        return null;
    }

    private E findParentState(E state) {
        for(Map.Entry<E, List<FlagState<E>>> entry: flagStates.entrySet()) {
            if(entry.getValue().stream().map(FlagState::getState).collect(Collectors.toList()).contains(state)) return entry.getKey();
        }

        //Simply return null if it's not found (this function should only be run once it has been verified that the input state is a flag state
        return null;
    }

    /**
     * Cancel any transition currently running
     * @return whether there was a transition to cancel
     */
    public final boolean cancelTransition() {
        if(transitioning && currentCommand != null) {
            currentCommand.cancel();
            currentState = currentTransition.getInterruptionState();
            return true;
        }
        return false;
    }

    private void outputErrorMessage(String message, String... args) {
        System.out.println("-----" + message + "!!!!-----");
        for(String a : args) {
            System.out.println(a);
        }
        System.out.println("-------------------------------------------------------");
    }

    private List<E> getStartStates() {
        return transitions.stream().map(Transition::getStartState).collect(Collectors.toList());
    }

    private List<E> getEndStates() {
        return transitions.stream().map(Transition::getEndState).collect(Collectors.toList());
    }

    private List<E> getInterruptionStates() {
        return transitions.stream().map(Transition::getInterruptionState).collect(Collectors.toList());
    }

    /**
     * Get a set of all states that have been marked as flag states
     * @return all states marked as flags
     */
    private Set<E> getStatesMarkedAsFlag() {
        Set<E> states = new HashSet<>();

        for(List<FlagState<E>> flagState : flagStates.values()) {
            states.addAll(flagState.stream().map(FlagState::getState).collect(Collectors.toSet()));
        }

        return states;
    }


    private Set<E> getStatesMarkedAsParent() {
        return flagStates.keySet();
    }

    /**
     * Determine whether the subsystem is actively running a command to transition between states
     * @return transitioning
     */
    public final boolean isTransitioning() {return transitioning;}

    /**
     * Find the current state the subsystem is in
     * @return Either the current state, or the current flag state (if there is a flag state)
     */
    public final E getCurrentState() {return flagState != null ? flagState : currentState;}

    /**
     * Find the current state the subsystem is trying to reach
     * @return the state the robot is trying to enter
     */
    public E getDesiredState() {return desiredState;}

    /**
     * Check if a subsystem is still in its Undetermined state
     * @return Whether the subsystem has determined itself
     */
    public boolean isUndetermined() {return currentState == undeterminedState;}

    /**
     * Get the Entry State of the subsystem (the state the subsystem will enter immediately after determining itself
     * @return the subsystem's entry state
     */
    public E getEntryState() { return entryState;}

    /**
     * Determine whether a subsystem is in a specific state...
     * @param state the state to check
     * @return Whether the subsystem is either in the given state or child flag state
     */
    public final boolean isInState(E state) {return currentState == state || flagState == state;}

    /**
     * Determine whether a subsystem is in any of the passed in states
     * @param states the state to check
     * @return Whether the subsystem is either in the given state or child flag state
     */
    public final boolean isInState(E... states) {
        for(E state : states) {
            if(currentState == state || flagState == state) return true;
        }

        return false;
    }

    /**
     * Get the current parent state the subsystem is in
     * @return the parent state
     */
    public final E getParentState() {return currentState;}

    /**
     * Get the flag state the subsystem is in (if any)
     * @return the flag state; Note: This value can be null
     */
    public final E getFlagState() {return flagState;}

    /**
     * @return The name by which you'd like the subsystem to be represented
     */
    public abstract String getName();

    /**
     * A command that waits indefinitely for a state to arrive and cancels whatever transition is ongoing if interrupted
     * @param state the state to wait for
     * @return Command from the factory
     */
    public Command waitForState(E state) {
        return new FunctionalCommand(() -> {}, () -> {}, (interrupted) -> {if(interrupted) cancelTransition();}, () -> isInState(state));
    }

    /**
     * A command that actively requests a state transition
     * @param state target state
     * @return command to be run
     */
    public Command goToStateCommand(E state) {
        return new FunctionalCommand(() -> {requestTransition(state);}, () -> {}, (interrupted) -> {if(interrupted) cancelTransition();}, () -> getCurrentState() == state);
    }

    /**
     * A command that actively requests a state transition for an instance-based State
     * @param state target state
     * @param command the continuous command that should be run once the transition is complete
     * @return command to be run
     */
    public Command goToStateCommand(E state, Command command) {
        return new FunctionalCommand(() -> {requestTransition(state, command);}, () -> {}, (interrupted) -> {if(interrupted) cancelTransition();}, () -> getCurrentState() == state);
    }

    /**
     * Variation of goToStateCommand() for having a delay
     * @param state target state
     * @param seconds delay
     * @return command to be run
     */
    public Command goToStateCommandDelayed(E state, double seconds) {
        return new WaitCommand(seconds).andThen(goToStateCommand(state));
    }

    /**
     * Variation of goToStateCommandDelayed() with a default delay of 20ms
     * @param state target state
     * @return command to be run
     */
    public Command goToStateCommandDelayed(E state) {
        return goToStateCommandDelayed(state, 0.02);
    }

    /**
     * Variation of goToStateCommand() for instance-based commands that includes a delay
     * @param state target state
     * @param command instance-based command
     * @param seconds delay
     * @return command to be run
     */
    public Command goToStateCommandDelayed(E state, Command command, double seconds) {
        return new WaitCommand(seconds).andThen(goToStateCommand(state, command));
    }

    /**
     * Variation of goToStateCommandDelayed() for instance-based commands with a default delay of 20ms
     * @param state target state
     * @param command instance-based command
     * @return command to be run
     */
    public Command goToStateCommandDelayed(E state, Command command) {
        return goToStateCommandDelayed(state, command, 0.02);
    }

    /**
     * Run an instantaneous transition, which is primarily meant for when the subsystem has just been enabled or disabled to instantly return to an idle state.
     * This happens immediately, regardless of current state of the subsystem
     * @param targetState the state to transition to
     * @param toRun runnable that will run before the transition resolves
     */
    public void runInstantaneousTransition(E targetState, Runnable toRun) {
        toRun.run();

        if(currentCommand != null) {
            currentCommand.cancel();
            currentCommand = null;
        }

        currentTransition = null;
        flagState = null;

        currentState = targetState;
    }

    public void rescheduleContinuousCommand() {transitioning = true; currentCommand = new InstantCommand();}


    /**
     * Tell the subsystem whether the subsystem is enabled or not
     * NOTE: Just because a SUBSYSTEM is enabled, that doesn't mean that the ROBOT is enabled
     * I.e. A subsystem for managing LEDs, to which data can be sent even while the robot is disabled
     * @param enabled whether the subsystem should be enabled or not
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if(enabled) onEnable();
        else onDisable();
    }

    /**
     * Enables the subsystem
     */
    public void enable() {setEnabled(true);}

    /**
     * Disables the subsystem
     */
    public void disable() {
        setEnabled(false);
        cancelTransition();
    }

    @Override
    public final void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.setSmartDashboardType("Stated subsystem");

        builder.addStringProperty("Name", () -> getName(), null);
        builder.addStringProperty("Current State", () -> getCurrentState() != null ? getCurrentState().name() : "null", null);
        builder.addStringProperty("Desired State", () -> getDesiredState() != null ? getDesiredState().name() : "null", null);
        builder.addStringProperty("Current Parent State", () -> getParentState() != null ? getParentState().name() : "null", null);
        builder.addStringProperty("Current Flag State", () -> getFlagState()!= null ? getFlagState().name() : "null", null);
        builder.addBooleanProperty("Transitioning", () -> isTransitioning(), null);
        builder.addBooleanProperty("Enabled", () -> enabled, null);

        additionalSendableData(builder);
    }

    protected abstract void additionalSendableData(SendableBuilder builder);

    /**
     * Override this method to do something when the robot enables
     */
    protected void onEnable() {}

    /**
     * Override this method to do something when the robot disables
     */
    protected void onDisable() {}

    /**
     * Override this method to add additional sendables to a subsystem
     * @return Map of Keys and values that will be sent when a Subsystem is registered in the Subsystem Manager
     */
    public Map<String, Sendable> additionalSendables() {return new HashMap<>();}
}
