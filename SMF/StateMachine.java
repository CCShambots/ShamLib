package frc.robot.ShamLib.SMF;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShamLib.SMF.graph.DirectionalEnumGraph;
import frc.robot.ShamLib.SMF.transitions.CommandTransition;
import frc.robot.ShamLib.SMF.transitions.TransitionBase;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

public abstract class StateMachine<E extends Enum<E>> extends SubsystemBase {
    private final DirectionalEnumGraph<E, TransitionBase<E>> transitionGraph;
    private TransitionBase<E> currentTransition;
    private TransitionBase<E> queuedTransition;
    private final HashMap<E, Command> stateCommands;
    private final Timer transitionTimer;
    private final Set<E> currentFlags;
    private final double transitionTimeOut = 2; //TODO: move to SMFConstants
    private final E undeterminedState;
    private E currentState;
    private boolean enabled;

    public StateMachine(String name, E undeterminedState, Class<E> enumType) {
        this.undeterminedState = undeterminedState;
        currentState = undeterminedState;
        currentTransition = null;
        transitionTimer = new Timer();
        currentFlags = new HashSet<>();
        stateCommands = new HashMap<>();
        setName(name);

        // *puke*
        transitionGraph = new DirectionalEnumGraph<>(enumType);
        enabled = false;
    }

    public E getState() {
        return currentState;
    }

    public void enable() {
        determineState();
        enabled = true;
    }

    public void disable() {
        enabled = false;
        currentTransition.cancel();
        currentTransition = null;
        queuedTransition = null;
        setState(undeterminedState);
        getCurrentCommand().cancel();
    }

    public boolean isDetermined() {
        return currentState != undeterminedState;
    }

    public void registerStateCommand(E state, Command c) {
        if (!stateCommands.containsKey(state)) stateCommands.put(state, c);
    }

    public void addTransition(E start, E end, Command run) {
        transitionGraph.addEdge(new CommandTransition<>(start, end, run));
    }

    public void addOmiTransition(E start, E end, Command run) {
        transitionGraph.addEdge(new CommandTransition<>(start, end, run));
        transitionGraph.addEdge(new CommandTransition<>(end, start, run));
    }

    public void addComutativeTransition(E start, E end, Command run1, Command run2) {
        transitionGraph.addEdge(new CommandTransition<>(start, end, run1));
        transitionGraph.addEdge(new CommandTransition<>(end, start, run2));
    }

    public boolean isTransitioning() {
        return currentTransition != null;
    }

    public TransitionBase<E> getCurrentTransition() {
        return currentTransition;
    }

    public void queueTransition(E state) {
        queuedTransition = transitionGraph.getEdge(currentState, state);
    }

    public Set<E> getCurrentFlags() {
        return currentFlags;
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

    public void reset() {
        if (currentTransition != null) {
            currentTransition.cancel();
            currentTransition = null;
        }
        setState(undeterminedState);
    }

    @Override
    public void periodic() {
        if (enabled) {
            updateTransitioning();

            if (currentTransition != null && !currentTransition.hasStarted()) currentTransition.execute();
        }

        update();
    }

    protected void setState(E state) {
        getCurrentCommand().end(true);
        currentState = state;
        clearFlags();
        if (stateCommands.containsKey(state)) stateCommands.get(state).schedule();
    }

    private void updateTransitioning() {
        if (currentTransition != null && currentTransition.isFinished()) {
            setState(currentTransition.getEndState());
            currentTransition = null;
        }

        if (queuedTransition != null && (!isTransitioning() || transitionTimer.hasElapsed(transitionTimeOut))) {
            forceChangeTransition();
        }
    }

    private void forceChangeTransition() {
        if (currentTransition == null) {
            currentTransition = queuedTransition;
            transitionTimer.reset();
            clearFlags();
            return;
        }

        currentTransition.cancel();
        currentTransition = queuedTransition;
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

    //TODO: logging stuff
}