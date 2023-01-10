package frc.robot.ShamLib.SMF.transitions;

import frc.robot.ShamLib.SMF.states.StateBase;

import java.util.function.BooleanSupplier;


public class InstantTransition<E extends Enum<E>> extends TransitionBase<E> {
    private Runnable toRun;

    /**
     * Create a transition that will start and finish instantaneously
     * @param startState transition start state
     * @param endState transition end state
     * //@param interruptionState transition interruption state
     * @param toRun lambda to run
     */
    public InstantTransition(StateBase<E> startState, StateBase<E> endState, Runnable toRun) {
        super(startState, endState);
        this.toRun = toRun;
    }

    @Override
    public String toString() {
        return "Start state: " + this.startState.getValue().name() + ", End state: " + this.endState.getValue().name() + ", Command: " + this.toRun.toString();
    }

    public Runnable getRunnable() {return toRun;}

    @Override
    public BooleanSupplier execute() {
        toRun.run();
        return () -> true;
    }

    @Override
    public void cancel() {} //Cancel doesn't need to do anything
}
