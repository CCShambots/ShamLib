package frc.robot.ShamLib.SMF;


public class SimpleTransition<E extends Enum<E>> {

    public E startState;
    public E endState;
    public E interruptionState;

    /**
     * Constructor for a transition that can be used for comparing transiitons(i.e basically storing states as a tuple instead of actually including a command)
     * @param startState beginning state
     * @param endState ending state
     * @param interruptionState state to go to in case of the command being interrupted
     */
    public SimpleTransition(E startState, E endState, E interruptionState) {
        this.startState = startState;
        this.endState = endState;
        this.interruptionState = interruptionState;
    }

    /**
     * Alternate constructor for the SimpleTransition that doesn't require an interruption state
     * @param startState beginning state
     * @param endState ending state
     */
    public SimpleTransition(E startState, E endState) {
        this(startState, endState, startState);
    }

    /**
     * Override of default equals() method to check for SimpleTransitions that have the same properties as the
     * @param obj the object against which to compare, (safe even for non SimpleTransition objects)
     * @return whether the two SimpleTransitions have the same properties
     */
    @Override
    public boolean equals(Object obj) {
        if(!(obj instanceof SimpleTransition)) return false;
        SimpleTransition<?> compare = (SimpleTransition<?>) obj;
        return compare.startState == this.startState && compare.endState == this.endState && compare.interruptionState == this.interruptionState;
    }

}
