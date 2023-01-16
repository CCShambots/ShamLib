package frc.robot.ShamLib.SMF;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShamLib.SMF.graph.DirectionalEnumGraph;
import frc.robot.ShamLib.SMF.transitions.CommandTransition;
import frc.robot.ShamLib.SMF.transitions.TransitionBase;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import static frc.robot.ShamLib.SMFConstants.SMF.transitionTimeout;

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
    private Class<E> enumType;

    public StateMachine(String name, E undeterminedState, Class<E> enumType) {
        this.enumType = enumType;

        this.undeterminedState = undeterminedState;
        currentState = undeterminedState;
        currentTransition = null;
        transitionTimer = new Timer();
        currentFlags = new HashSet<>();
        stateCommands = new HashMap<>();
        setName(name);

        transitionGraph = new DirectionalEnumGraph<>(enumType);
        enabled = false;
    }

    public E getState() {
        return currentState;
    }

    public void enable() {
        determineState();
        enabled = true;

        onEnable();
    }

    protected abstract void onEnable();

    public void disable() {
        enabled = false;
        if (currentTransition != null) currentTransition.cancel();
        currentTransition = null;
        queuedTransition = null;
        setState(undeterminedState);
        if (getCurrentCommand() != null) getCurrentCommand().cancel();

        onDisable();
    }

    protected abstract void onDisable();

    public boolean isDetermined() {
        return currentState != undeterminedState;
    }

    public void registerStateCommand(E state, Command c) {
        if (!stateCommands.containsKey(state)) stateCommands.put(state, c);
    }

    public void addTransition(E start, E end, Command run) {
        transitionGraph.addEdge(new CommandTransition<>(start, end, run));
    }

    public void addOmniTransition(E state, Command run) {
        for (E s : enumType.getEnumConstants()) {
            addTransition(state, s, run);
        }
    }

    public void addCommutativeTransition(E start, E end, Command run) {
        transitionGraph.addEdge(new CommandTransition<>(start, end, run));
        transitionGraph.addEdge(new CommandTransition<>(end, start, run));
    }

    public void addCommutativeTransition(E start, E end, Command run1, Command run2) {
        transitionGraph.addEdge(new CommandTransition<>(start, end, run1));
        transitionGraph.addEdge(new CommandTransition<>(end, start, run2));
    }

    public boolean isTransitioning() {
        return currentTransition != null;
    }

    public TransitionBase<E> getCurrentTransition() {
        return currentTransition;
    }

    /**
     * Request a transition to a state
     * @param state state to transition to
     */
    public void requestTransition(E state) {
        TransitionBase<E> transition = transitionGraph.getEdge(currentState, state);
        if(!isTransitioning() && transition != null) {
            currentTransition = transition;
            transition.execute();
            transitionTimer.start();

            updateTransitioning();
        } else {
            queuedTransition = transition;
        }

    }

    public void requestTransition(E state, Command command) {
        stateCommands.put(state, command);
        requestTransition(state);
    }

    public Command transitionCommand(E state) {
        return new FunctionalCommand(() -> requestTransition(state),
                () -> {}, (interrupted) -> {}, () -> getState() == state);
    }

    public Command transitionCommand(E state, Command command) {
        return new FunctionalCommand(() -> requestTransition(state, command),
                () -> {}, (interrupted) -> {}, () -> getState() == state);
    }

    public Set<E> getCurrentFlags() {
        return currentFlags;
    }

    public boolean isFlag(E state) {
        return getCurrentFlags().contains(state);
    }

    public void setFlag(E flag) {
        currentFlags.add(flag);
    }

    public void clearFlag(E flag) {
        currentFlags.remove(flag);
    }

    public void clearFlags() {
        currentFlags.clear();
    }


    @Override
    public void periodic() {
        if (enabled) {
            updateTransitioning();
        }

        update();
    }

    protected void setState(E state) {
        if (getCurrentCommand() != null) getCurrentCommand().cancel();
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

    public String toString() {
        return "Stated Subsystem Machine - " + getName() + "; In State: " + getState().name() + "; In Transition: " + isTransitioning();
    }
    public void determineState() {
        if (!isDetermined()) determineSelf();
    }
    protected abstract void update();
    protected abstract void determineSelf();


    @Override
    public final void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.setSmartDashboardType("Stated subsystem");

        builder.addStringProperty("Name", () -> getName(), null);
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
        builder.addBooleanProperty("Transitioning", () -> isTransitioning(), null);
        builder.addBooleanProperty("Enabled", () -> enabled, null);

        additionalSendableData(builder);
    }

    protected abstract void additionalSendableData(SendableBuilder builder);

}
