package frc.robot.ShamLib.SMF;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ShamLib.SMF.exceptions.*;
import frc.robot.ShamLib.SMF.exceptions.FlagStateException.FlagStateReason;
import frc.robot.ShamLib.SMF.exceptions.InvalidTransitionException.TransitionReason;
import frc.robot.ShamLib.SMF.exceptions.TransitionException.TransitionExceptionReason;
import frc.robot.ShamLib.SMF.graph.DirectionalGraph;
import frc.robot.ShamLib.SMF.graph.exceptions.ExistingEdgeException;
import frc.robot.ShamLib.SMF.states.*;
import frc.robot.ShamLib.SMF.transitions.CommandTransition;
import frc.robot.ShamLib.SMF.transitions.InstantTransition;
import frc.robot.ShamLib.SMF.transitions.TransitionBase;

import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

public abstract class StateMachine<E extends Enum<E>> implements Sendable {

    private DirectionalGraph<StateBase<E>, TransitionBase<E>, E> stateGraph;

    private StateBase<E> entryState;
    private StateBase<E> currentState;
    private StateBase<E> flagState;
    private StateBase<E> desiredState;

    private StateBase<E> startingState;

    private TransitionBase<E> currentTransition;
    private boolean transitioning = false;
    private BooleanSupplier transitionFinished = () -> false;
    private Command currentCommand = null;
    private boolean enabled = false;

    private Mode currentMode = Mode.Standard;
    private StateMachine<?> currentSubMachine;

    private List<TransitionBase<E>> transitions;
    private Set<E> startStates;
    private Set<E> endStates;
    private Set<FlagState<E>> flagStates;
    private Set<SubmachineState<E>> submachineStates;
    private Set<E> parentStates;
    private Set<ContinuousState<E>> continuousStates;
    private Set<InstanceBasedState<E>> instanceBasedStates;

    public StateMachine(E startingState) {        
        this.startingState = stateGraph.findOrCreateVertex(startingState).getValue();
        this.currentState = stateGraph.findOrCreateVertex(startingState).getValue();

        stateGraph = new DirectionalGraph<>(StandardState::new, () -> {
            transitions = stateGraph.getEdges().stream().map((e) -> e.getValue()).collect(Collectors.toList());

            startStates = computeStartStates();
            endStates = computeEndStates();
            flagStates = computeFlagStates();
            submachineStates = computeSubmachineStates();
            parentStates = computeParentStates();
            continuousStates = computeContinuousCommandStates();
            instanceBasedStates = computeInstanceBasedStates();
        });
    }

    /**
     * Creates a CommandTransition back and forth between two states
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
     * Creates an InstantTransition back and forth between two states
     * @param state1 One state
     * @param state2 The other state
     * @param toRun1 The runnable to run between state1 to state2
     * @param toRun2 The runnable to run between state2 to state1
     */
    protected void addCommutativeTransition(E state1, E state2, Runnable toRun1, Runnable toRun2) {
        addTransition(state1, state2, toRun1);
        addTransition(state2, state1, toRun2);
    }

    /**
     * Create a CommandTransition between / among many states easily. A transition with the same command will be created going from every provided start state
     * to every provided end state
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
     * Create CommandTransitions to one end state from many start states.
     * @param startStates the states which can go to the end state
     * @param endState the one state that you are going towards
     * @param command The command to be run for the transition
     */
    protected void addMultiTransitions(E[] startStates, E endState, Command command) {
       for(E state : startStates) {
           addTransition(state, endState, command);
       }
    }

    /**
     * Create CommandTransitions to one end state from many start states.
     * @param startState the state to start at
     * @param endStates the states at which the transition can end
     * @param command The command to be run for the transition
     */
    protected void addMultiTransitions(E startState, E[] endStates, Command command) {
       for(E state : endStates) {
           addTransition(startState, state, command);
       }
    }

    /**
     * Create an InstantTransition between / among many states easily. A transition with the same command will be created going from every provided start state
     * to every provided end state
     * @param startingStates all starting states to apply the transitions to
     * @param endingStates all ending states to apply the transitions to
     * @param toRun the runnable to run
     */
    protected void addOmniTransition(E[] startingStates, E[] endingStates, Runnable toRun) {
        for(E start : startingStates) {
            for (E end : endingStates) {
                addTransition(start, end, toRun);
            }
        }
    }

    /**
     * Create InstantTransitions to one end state from many start states.
     * @param startStates the states which can go to the end state
     * @param endState the one state that you are going towards
     * @param toRun The runnable to be run
     */
    protected void addMultiTransitions(E[] startStates, E endState, Runnable toRun) {
       for(E state : startStates) {
           addTransition(state, endState, toRun);
       }
    }

    /**
     * Create InstantTransitions from one start state to many end states.
     * @param startState the state to start at
     * @param endStates the states at which the transition can end
     * @param toRun The runnable to be run
     */
    protected void addMultiTransitions(E startState, E[] endStates, Runnable toRun) {
       for(E state : endStates) {
           addTransition(startState, state, toRun);
       }
    }


    /**
     * Creates a CommandTransition between two states
     * @param startState where the machine begins the transition
     * @param endState where the subsystem ends the transition
     * @param command the command that runs
     */
    protected void addTransition(E  startState, E endState, Command command) {
        addTransition(new CommandTransition<E>(startState, endState, command));
    }

    /**
     * Creates an InstantTransition between two states
     * @param startState where the machine begins the transition
     * @param endState where the machine ends the transition
     * @param toRun the runnable to run
     */
    protected void addTransition(E startState, E endState, Runnable toRun) {
        addTransition(new InstantTransition<E>(startState, endState, toRun));
    }


    /**
     * Alternate version of the method where an interruption state can be stated
     * @param t the transition to add
     * @return whether adding the transition was successful
     */
    protected boolean addTransition(TransitionBase<E> t) {

        //End the function if the transition conflicts
        if(!checkIfValidTransition(t)) return false;

        stateGraph.findOrCreateVertex(t.getStartValue()).getValue().setPartOfTransition(true);
        stateGraph.findOrCreateVertex(t.getEndValue()).getValue().setPartOfTransition(true);

        try {
            stateGraph.addEdge(t.getStartValue(), t.getEndValue(), t);
            return true;
        } catch (ExistingEdgeException e) {
            new InvalidTransitionException(getName(), t, TransitionReason.DuplicateTransition).printStackTrace();
        }

        return false;
    }


    /**
     * Add a new flag state to better indicate the subsystem's state
     * If a subsystem fulfills two flag states at once, the one it displays will be unreliable
     * NOTE: If you add two of the same flag state, unexpected behavior will occur
     * @param parentValue the parent state in which the flag state should trigger
     * @param flagValue the flag state which can potentially be active
     * @param condition under what conditions the flag state should be active
     * @return whether the flag state was added successfully
     */
    protected boolean addFlagState(E parentValue, E flagValue, BooleanSupplier condition) {
        try {
            StateBase<E> flagState = stateGraph.findOrCreateVertex(flagValue).getValue();
            StateBase<E> parentState = stateGraph.findOrCreateVertex(parentValue).getValue();

            //If the flag state is already registered as part of transitions, it is invalid
            if(flagState.isPartOfTransition() || parentState.isPartOfTransition()) {
                throw new FlagStateException(getName(), flagValue.name(), parentValue.name(), FlagStateReason.PartOfTransition);
            }
    
            //Any state marked as a flag state cannot be a parent state
            if(getFlagStates().contains(parentState)) {
                throw new FlagStateException(getName(), flagValue.name(), parentValue.name(), FlagStateReason.FlagAlreadyParent);
            }
    
            //Any state already marked as a parent state cannot become a flag state
            if(getParentStates().contains(flagState)) {
                throw new FlagStateException(getName(), flagValue.name(), parentValue.name(), FlagStateReason.ParentAlreadyFlag);
            }

            if(getSubmachineStates().contains(flagState)) {
                throw new FlagStateException(getName(), flagValue.name(), parentValue.name(), FlagStateReason.AlreadySubmachine);
            }
    
            //Register the vertex as a flag state
            stateGraph.findOrCreateVertex(flagValue).setValue(new FlagState<>(flagValue, parentValue, condition));

            return true;

        }catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    /**
     * Set a command to schedule once a state is reached.
     * This command should run indefinitely (i.e. isFinished() should never return true).
     * The command will be canceled when a transition begins.
     * Only the first command registered for a certain state will be
     * @param stateValue the state that should have a continuous command
     * @param command the command that should run once the subsystem reaches that state
     * @return whether the command was successfully added
     */
    protected boolean setContinuousCommand(E stateValue, Command command) {
        try {
            StateBase<E> state = stateGraph.findOrCreateVertex(stateValue).getValue();
            StateBase<E> proposedState = new ContinuousState<>(stateValue, command);
            if (isFlagState(state)) {
                throw new IncompatibleStateException(this, state, proposedState);
            } else if(isContinuousCommandState(state)){
                throw new IncompatibleStateException(this, state, proposedState);
            } else if(isInstanceBasedState(state)) {
                throw new IncompatibleStateException(this, state, proposedState);
            }else if(isSubmachineState(stateValue)) {
                throw new IncompatibleStateException(this, state, proposedState);
            }
            else stateGraph.findOrCreateVertex(stateValue)
                        .setValue(proposedState);

            return true;
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    protected boolean addSubmachineState(E stateValue, StateMachine<?> machine) {
        try {
            StateBase<E> state = stateGraph.findOrCreateVertex(stateValue).getValue();
            StateBase<E> proposedState = new SubmachineState<>(stateValue, machine);
            if(isFlagState(state)) {
                throw new IncompatibleStateException(this, state, proposedState);
            }else if(isContinuousCommandState(state)) {
                throw new IncompatibleStateException(this, state, proposedState);
            } else if(isInstanceBasedState(state)) {
                throw new IncompatibleStateException(this, state, proposedState);
            }
            stateGraph.findOrCreateVertex(stateValue)
                    .setValue(proposedState);

            return true; 
        }catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    /**
     * Define a state that will start a different continuous command each time it is reached
     * This state cannot be a flag state, or a state with a continuous command
     * @param stateValue the state that should become instance-based
     * @return whether the state was successfully marked as instance-based
     */
    protected boolean addInstanceBasedState(E stateValue) {
        try {
            StateBase<E> state = stateGraph.findOrCreateVertex(stateValue).getValue();
            StateBase<E> proposedState = new InstanceBasedState<>(stateValue);
            if(isFlagState(state)) throw new IncompatibleStateException(this, state, proposedState);
    
            if(isContinuousCommandState(state)) throw new IncompatibleStateException(this, state, proposedState);

            return true;
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }



    /**
     * @param proposedTransition The transition that should be compared against all the other transitions
     * @return whether the proposed transition is valid or not
     */
    private boolean checkIfValidTransition(TransitionBase<E> proposedTransition) {
        try {

            //The transition is invalid if either one of its states have been marked as flag states
            if(isFlagState(proposedTransition.getStartValue())) {
                throw new IncompatibleStateException(this, stateGraph.findOrCreateVertex(proposedTransition.getStartValue()).getValue(), proposedTransition.getStartState());
            }
            if(isFlagState(proposedTransition.getEndValue())) {
                throw new IncompatibleStateException(this, stateGraph.findOrCreateVertex(proposedTransition.getEndValue()).getValue(), proposedTransition.getEndState());
            }

            //Check the other defined transitions to see if there are any conflicts (if they represent the same states, or if they
            for(TransitionBase<E> t : getTransitions()) {
                if(!proposedTransition.isValid(t)) {
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
     * Method for controlling the state of the state machine
     */
    public final void update() {

        //States can only be managed whilst the subsystem is enabled
        if(enabled) {

            if(currentMode == Mode.Submachine) {
                if(currentSubMachine != null) {
                    currentSubMachine.update();
                }
            }

            if(transitioning && transitionFinished.getAsBoolean()) {
                finishTransition();
            }

            //Check for flag states and activate exactly one of them
            //There is only one flag position potentially active at a time
            //Also negate the existing flag state if its condition is no longer true
            if(getParentStates().contains(currentState)) {
                for(FlagState<E> f : currentState.getFlagStates()) {
                    if(f.getCondition().getAsBoolean()) {
                        flagState = f;
                    } else if(f == flagState) {
                        flagState = null;
                    }
                }
            }
        }
    }

    /**
     * Takes a finished transition and updates the current robot state.
     * It searches for a continuous command to schedule
     */
    private void finishTransition() {
        transitioning = false;
        currentState = desiredState;
        flagState = null;

        if(currentState instanceof ContinuousState) {
            currentCommand = ((ContinuousState<E>) currentState).getCommand();
        }

        if(currentState instanceof InstanceBasedState) {
            ((InstanceBasedState<E>) currentState).clearCommand();
        }
        
        //Schedule the command if one is found
        if(currentCommand != null) {
            currentCommand.schedule();
        }
    }

    /**
     * @param state The state to which the subsystem should go
     * @return whether a transition was successfully found
     */
    public boolean requestTransition(E state) {
        try {
    
            if(!getEndStates().contains(state)) {
                throw new TransitionException(getName(), state.name(), TransitionExceptionReason.MissingEndState);
            }
    
            TransitionBase<E> t = findTransition(currentState.getValue(), state);
    
            //If a valid transition was found, then start performing it
            if(t != null) {
    
                //If a transition is already occurring, cancel it
                if(transitioning) {
                    currentTransition.cancel();
                }
    
                //If a continuous command is running, cancel it
                if(!transitioning && currentCommand != null) {
                    currentCommand.cancel();
                }
    
                desiredState = state;
                currentTransition = t;
                transitioning = true;

                transitionFinished = t.execute();

                if(transitionFinished.getAsBoolean()) {
                    finishTransition();
                }
            } else return false;
    
            return true;
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    /**
     * Request that a subsystem move to a target state (for instance-based commands)
     * @param state the desired state the subsystem should go to
     * @param command the command that should be run for this instance
     * @return whether a transition was successfully found
     */
    public boolean requestTransition(E state, Command command) {
        try {
            if(perInstanceStates.contains(state)) {
                continuousCommands.put(state, command);
            } else throw new TransitionException(getName(), state.name(), TransitionExceptionReason.NotInstanceState);
    
            return requestTransition(state);
            
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    /**
     * Search the list of transitions to find which transition should be implemented
     * @return Whatever transition is found. This can be null
     */
    private TransitionBase<E> findTransition(E startState, E endState) {
        for(TransitionBase<E> t : getTransitions()) {
            if(startState==t.getStartValue() && endState==t.getEndValue()) return t;
        }

        return null;
    }


    private FlagState<E> findParentState(E state) {
        for(FlagState<E> entry : getFlagStates()) {
            if(entry.getValue() == state) return entry;
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
            currentState = currentTransition.getStartState();
            return true;
        }
        return false;
    }

    //TODO: Precompute these upon each change to the graph
    private Set<E> computeStartStates() {
        return stateGraph.getEdges().stream().map((e) -> e.getValue().getStartValue()).collect(Collectors.toSet());
    }

    private Set<E> computeEndStates() {
        return stateGraph.getEdges().stream().map((e) -> e.getValue().getEndValue()).collect(Collectors.toSet());
    }

    private Set<FlagState<E>> computeFlagStates() {
        return stateGraph.getVertexObjectsWithCondition((e) -> e instanceof FlagState)
                .stream().map((e) -> (FlagState<E>) e).collect(Collectors.toSet());
    }

    private Set<SubmachineState<E>> computeSubmachineStates() {
        return stateGraph.getVertexObjectsWithCondition((e) -> e instanceof SubmachineState)
                .stream().map((e) -> (SubmachineState<E>) e).collect(Collectors.toSet());
    }

    private Set<E> computeParentStates() {
        return getFlagStates().stream().map(FlagState::getParentState).collect(Collectors.toSet());
    }

    private Set<ContinuousState<E>> computeContinuousCommandStates() {
        return stateGraph.getVertexObjectsWithCondition((e) -> e instanceof ContinuousState)
                .stream().map((e) -> (ContinuousState<E>) e).collect(Collectors.toSet());
    }

    private Set<InstanceBasedState<E>> computeInstanceBasedStates() {
        return stateGraph.getVertexObjectsWithCondition((e) -> e instanceof InstanceBasedState)
                .stream().map((e) -> (InstanceBasedState<E>) e).collect(Collectors.toSet());
    }

    public List<TransitionBase<E>> getTransitions() {
        return transitions;
    }

    public Set<E> getStartStates() {
        return startStates;
    }

    public Set<E> getEndStates() {
        return endStates;
    }

    public Set<FlagState<E>> getFlagStates() {
        return flagStates;
    }

    public Set<SubmachineState<E>> getSubmachineStates() {
        return submachineStates;
    }

    public Set<E> getParentStates() {
        return parentStates;
    }

    public Set<ContinuousState<E>> getContinuousStates() {
        return continuousStates;
    }

    public Set<InstanceBasedState<E>> getInstanceBasedStates() {
        return instanceBasedStates;
    }

    private boolean isStartState(E state) {
        return getStartStates().contains(state);
    }

    private boolean isEndState(E state) {
        return getEndStates().contains(state);
    }

    private boolean isFlagState(E state) {
        return getFlagStates().stream().anyMatch((e) -> e.getValue() == state);
    }

    private boolean isFlagState(StateBase<E> state)  {
        return state instanceof FlagState;
    }

    private boolean isParentState(E state) {
        return getParentStates().contains(state);
    }

    private boolean isSubmachineState(E state) {
        return getSubmachineStates().stream().anyMatch((e) -> e.getValue() == state);
    }

    private boolean isSubmachineState(StateBase<E> state) {
        return state instanceof SubmachineState;
    }

    private boolean isContinuousCommandState(E state) {
        return getContinuousStates().stream().anyMatch((e) -> e.getValue() == state);
    }

    private boolean isContinuousCommandState(StateBase<E> state) {
        return state instanceof ContinuousState;
    }

    private boolean isInstanceBasedState(E state) {
        return getInstanceBasedStates().stream().anyMatch((e) -> e.getValue() == state);
    }

    private boolean isInstanceBasedState(StateBase<E> state) {
        return state instanceof InstanceBasedState;
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
    public final E getCurrentState() {
        return flagState != null ? flagState : currentState;
    }

    /**
     * Find the current state the subsystem is trying to reach
     * @return the state the robot is trying to enter
     */
    public E getDesiredState() {return desiredState;}

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
    public final boolean isInState(E state) {return currentState.getValue() == state || flagState.getValue() == state;}

    /**
     * Determine whether a subsystem is in any of the passed in states
     * @param states the state to check
     * @return Whether the subsystem is either in the given state or child flag state
     */
    public final boolean isInState(E... states) {
        for(E state : states) {
            if(currentState.getValue() == state || flagState.getValue() == state) return true;
        }

        return false;
    }

    /**
     * Get the current parent state the subsystem is in
     * @return the parent state
     */
    public final StateBase<E> getParentState() {return currentState;}

    /**
     * Get the flag state the subsystem is in (if any)
     * @return the flag state; Note: This value can be null
     */
    public final StateBase<E> getFlagState() {return flagState;}

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
     * Forces the subsystem to a specific state
     * @param targetState the state to transition to
     * @param toRun runnable that will run before the transition resolves
     */
    public void forceState(E targetState, Runnable toRun) {
        toRun.run();

        if(currentCommand != null) {
            currentCommand.cancel();
            currentCommand = null;
        }

        flagState = null;

        currentState = targetState;
    }

    public void rescheduleContinuousCommand() {transitioning = true; currentCommand = new InstantCommand();}

    /**
     * Reset the state machine by going back to the starting state of the subsystem and resetting the mode of the subsystem
     */
    public void reset() {
        forceState(startingState, () -> {
            currentMode = Mode.Standard;
        });
    }


    /**
     * Tell the subsystem whether the subsystem is enabled or not
     * NOTE: Just because a SUBSYSTEM is enabled, that doesn't mean that the ROBOT is enabled
     * I.e. A subsystem for managing LEDs, to which data can be sent even while the robot is disabled
     * @param enabled whether the subsystem should be enabled or not
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if(enabled) onEnable();
        else {
            currentTransition.cancel();
            onDisable();
        }
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

    public enum Mode {
        Standard, Submachine
    }

}
