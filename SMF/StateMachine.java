package frc.robot.ShamLib.SMF;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.ShamLib.SMF.graph.DirectionalEnumGraph;
import frc.robot.ShamLib.SMF.transitions.CommandTransition;
import frc.robot.ShamLib.SMF.transitions.TransitionBase;

import java.util.*;

import static frc.robot.ShamLib.ShamLibConstants.SMF.transitionTimeout;

public abstract class StateMachine<E extends Enum<E>> extends SubsystemBase {
    private final DirectionalEnumGraph<E, TransitionBase<E>> transitionGraph;
    private TransitionBase<E> currentTransition;
    private TransitionBase<E> queuedTransition;
    private final HashMap<E, Command> stateCommands;
    private final Timer transitionTimer;
    private final Set<E> currentFlags;
    private final double transitionTimeOut = transitionTimeout;
    private final E undeterminedState;
    private E currentState;
    private boolean enabled;
    private final Class<E> enumType;
    private final List<StateMachine<?>> subsystems;

    /**
     * Instantiate a new State Machine
     * @param name name of the state machine to send over network tables
     * @param undeterminedState the undetermined state of the subsystem
     * @param enumType the class of enums to use for the state
     */
    public StateMachine(String name, E undeterminedState, Class<E> enumType) {
        this.enumType = enumType;

        this.undeterminedState = undeterminedState;
        currentState = undeterminedState;
        currentTransition = null;
        transitionTimer = new Timer();
        currentFlags = new HashSet<>();
        stateCommands = new HashMap<>();
        subsystems = new ArrayList<>();
        setName(name);

        transitionGraph = new DirectionalEnumGraph<>(enumType);
        enabled = false;
    }

    /**
     * @return the child state-machines
     */
    public final List<StateMachine<?>> getChildSubsystems() {
        return subsystems;
    }

    /**
     * adds the specified system to the set of child subsystems owned by the current state machine
     * @param machine the subsystem to add as a child
     */
    protected final void addChildSubsystem(StateMachine<?> machine) {
        subsystems.add(machine);
    }

    /**
     * @return the current state of the machine
     */
    public final E getState() {
        return currentState;
    }

    /**
     * Enable the state machine to start running.
     * Transitions and state commands will only run while the machine is enabled
     */
    public final void enable() {
        determineState();
        enabled = true;

        onEnable();
    }

    /**
     * Whether the state machine is enabled
     * @return the current enabled status of the state machine
     */
    public final boolean isEnabled() {
        return enabled;
    }

    /**
     * User-implemented method run immediately on the machine being enabled
     */
    protected void onEnable() {}

    /**
     * User-implemented method ran immediately when the teleop period is started
     */
    protected void onTeleopStart() {}

    /**
     * User-implemented method ran immediately when the autonomous period is started
     */
    protected void onAutonomousStart() {}

    /**
     * Stop the state machine from running.
     * No transitions or state commands will run while the machine is disabled
     */
    public final void disable() {
        enabled = false;
        if (currentTransition != null) currentTransition.cancel();
        currentTransition = null;
        queuedTransition = null;
        setState(undeterminedState);
        if (getCurrentCommand() != null) getCurrentCommand().cancel();

        onDisable();
    }

    /**
     * User-implemented method run immediately upon the machine being disabled
     */
    protected void onDisable() {}

    /**
     * @return whether the machine is in a determined state
     */
    public final boolean isDetermined() {
        return currentState != undeterminedState;
    }

    /**
     * Set a state command to run upon reaching that state
     * @param state the state during which the command should run
     * @param command command to run
     */
    public final void registerStateCommand(E state, Command command) {
        stateCommands.put(state, command);
    }

    /**
     * Add an instant state command to run upon reaching that state
     * @param state the state during which the command should run
     * @param toRun the command to run
     */
    protected final void registerStateCommand(E state, Runnable toRun) {
        registerStateCommand(state, new InstantCommand(toRun));
    }

    /**
     * Create a new transition in the machine
     * @param start the start state of the transition
     * @param end the end state of the transition
     * @param command the command to run
     */
    protected final void addTransition(E start, E end, Command command) {
        transitionGraph.addEdge(new CommandTransition<>(start, end, command));
    }

    protected final void addTransition(E start, E end) {
        transitionGraph.addEdge(new CommandTransition<>(start, end, new InstantCommand()));
    }

    protected final void addTransition(E start, E end, Runnable toRun) {
        transitionGraph.addEdge(new CommandTransition<>(start, end, new InstantCommand(toRun)));
    }

    protected final void removeTransition(E start, E end) {
        transitionGraph.removeEdge(start, end);
    }

    /**
     * Adds a transition from every state to the given state
     * @param state the state to go to
     * @param run transition command to run
     */
    public final void addOmniTransition(E state, Command run) {
        for (E s : enumType.getEnumConstants()) {
            if(s != state) {
                addTransition(s, state, run);
            }
        }
    }

    /**
     * Adds a transition from every state to the given state
     * @param state the state to go to
     * @param run the transition runnable to run as an instant command
     */
    public final void addOmniTransition(E state, Runnable run) {
        addOmniTransition(state, new InstantCommand(run));
    }

    public final void addOmniTransition(E state) {
        addOmniTransition(state, () -> {});
    }

    /**
     * Add a transition both ways between two states
     * @param start beginning state
     * @param end ending state
     * @param run the command to run between them
     */
    public final void addCommutativeTransition(E start, E end, Command run) {
        transitionGraph.addEdge(new CommandTransition<>(start, end, run));
        transitionGraph.addEdge(new CommandTransition<>(end, start, run));
    }

    /**
     * Add a transition both ways between two states
     * @param start beginning state
     * @param end ending state
     * @param run1 the command to run between start and end
     * @param run2 the command to run between end and start
     */
    public final void addCommutativeTransition(E start, E end, Command run1, Command run2) {
        transitionGraph.addEdge(new CommandTransition<>(start, end, run1));
        transitionGraph.addEdge(new CommandTransition<>(end, start, run2));
    }

    /**
     * @return true if the machine is actively transitioning between states
     */
    public final boolean isTransitioning() {
        return currentTransition != null;
    }

    /**
     * @return the object that returns the current transition information. Will be null if there is no transition
     */
    public final TransitionBase<E> getCurrentTransition() {
        return currentTransition;
    }

    /**
     * Request a transition to a state
     * @param state state to transition to
     */
    public final void requestTransition(E state) {
        TransitionBase<E> transition = transitionGraph.getEdge(currentState, state);
        if(!isTransitioning() && transition != null) {
            currentTransition = transition;
            cancelStateCommand();
            transition.execute();
            transitionTimer.start();

            updateTransitioning();
        } else {
            queuedTransition = transition;
        }

    }

    private void cancelStateCommand() {
        if (stateCommands.containsKey(getState())) {
            Command prevCommand = stateCommands.get(getState());
            if (prevCommand.isScheduled()) prevCommand.cancel();
        }
    }

    /**
     * Request an instance-based state on the state machine
     * @param state the state to go to
     * @param command the command to run upon reaching the state
     */
    public final void requestTransition(E state, Command command) {
        stateCommands.put(state, command);
        requestTransition(state);
    }

    /**
     * A command that will keep running until it gets to the target state
     * @param state the state to go to
     * @return the command to run
     */
    public final Command transitionCommand(E state) {
        return new FunctionalCommand(() -> requestTransition(state),
                () -> {}, (interrupted) -> {}, () -> getState() == state);
    }

    /**
     * A command that will keep running until it gets to the target state, but it supports instance-based states
     * @param state the state to go to
     * @param command the command to run upon reaching that state
     * @return the command to run
     */
    public final Command transitionCommand(E state, Command command) {
        return new FunctionalCommand(() -> requestTransition(state, command),
                () -> {}, (interrupted) -> {}, () -> getState() == state);
    }

    /**
     * @param state the state to wait for
     * @return the command to run
     */
    public final Command waitForState(E state) {
        return new WaitUntilCommand(() -> getState() == state);
    }

    /**
     * Wait for the state machine to indicate that a flag has been found
     * @param flag the flag state to wait for
     * @return the command to run
     */
    public final Command waitForFlag(E flag) {
        return new WaitUntilCommand(() -> isFlag(flag));
    }

    /**
     * Get the current flags of a state
     * @return the current flags
     */
    public final Set<E> getCurrentFlags() {
        return currentFlags;
    }

    /**
     * Determine whether a state is currently an active flag state
     * @param state the state to evaluate
     * @return whether the state is currently a flag or not
     */
    public final boolean isFlag(E state) {
        return getCurrentFlags().contains(state);
    }

    /**
     * Add a flag state to the current state
     * @param flag the flag state
     */
    public final void setFlag(E flag) {
        currentFlags.add(flag);
    }

    /**
     * A command to set a flag on the state machine
     * @param flag the flag to set on the state machine
     * @return the command to run
     */
    public final Command setFlagCommand(E flag) {
        return new InstantCommand(() -> setFlag(flag));    
    }

    /**
     * Remove a specific flag state from the list of flags states
     * @param flag the flag state to clear
     */
    public final void clearFlag(E flag) {
        currentFlags.remove(flag);
    }

    /**
     * Clear all states that are currently flags
     */
    public final void clearFlags() {
        currentFlags.clear();
    }


    @Override
    public final void periodic() {
        if (enabled) {
            updateTransitioning();
        }

        update();
    }

    protected final void setState(E state) {
        cancelStateCommand();

        currentState = state;
        clearFlags();
        if (stateCommands.containsKey(state)) {
            stateCommands.get(state).schedule();
        }
    }

    private void updateTransitioning() {
        if (isTransitioning() && currentTransition.isFinished()) {
            setState(currentTransition.getEndState());
            currentTransition = null;
            transitionTimer.stop();
            transitionTimer.reset();
        }

        if (queuedTransition != null && (!isTransitioning() || transitionTimer.hasElapsed(transitionTimeOut))) {
            forceChangeTransition();
        }
    }

    private void forceChangeTransition() {

        if(currentTransition != null) currentTransition.cancel();
        currentTransition = queuedTransition;
        currentTransition.execute();
        queuedTransition = null;
        transitionTimer.reset();
        clearFlags();
    }

    public final String toString() {
        return "Stated Subsystem Machine - " + getName() + "; In State: " + getState().name() + "; In Transition: " + isTransitioning();
    }
    public final void determineState() {
        if (!isDetermined()) determineSelf();
    }
    protected void update() {}

    /**
     * User-implemented method to determine the state of the machine.
     * THIS METHOD IS RESPONSIBLE FOR CALLING the setState() method
     */
    protected abstract void determineSelf();


    @Override
    public final void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.setSmartDashboardType("Stated subsystem");

        builder.addStringProperty("Name", this::getName, null);
        builder.addStringProperty("Current State", () -> getState().name(), null);
        builder.addStringProperty("Desired State", () -> isTransitioning() ? getCurrentTransition().getEndState().name() : getState().name(), null);
        builder.addStringArrayProperty("Current Flag States", () -> {
            int n = getCurrentFlags().size();
            String arr[] = new String[n];

            int i = 0;
            for(E flag : getCurrentFlags()) {
                arr[i] = flag.toString();
                i++;
            }

            return arr;
        }, null);
        builder.addBooleanProperty("Transitioning", this::isTransitioning, null);
        builder.addBooleanProperty("Enabled", () -> enabled, null);

        additionalSendableData(builder);
    }

    protected void additionalSendableData(SendableBuilder builder) {}

    public Map<String, Sendable> additionalSendables() {
        return new HashMap<>();
    }
}
