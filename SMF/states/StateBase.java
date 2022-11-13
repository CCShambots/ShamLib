package frc.robot.ShamLib.SMF.states;

import java.util.List;
import java.util.ArrayList;

public abstract class StateBase<E extends Enum<E>> {
    private final E value;
    private boolean partOfTransition = false;

    private final List<FlagState<E>> flagStates = new ArrayList<>();

    public StateBase(E value) {
        this.value = value;
    }

    public boolean isPartOfTransition() {
        return partOfTransition;
    }

    public void setPartOfTransition(boolean partOfTransition) {
        this.partOfTransition = partOfTransition;
    }

    public E getValue() {
        return value;
    }

    public void addFlagState(FlagState<E> state) {flagStates.add(state);}

    public List<FlagState<E>> getFlagStates() {return flagStates;}

    @Override
    public String toString() {
        return
                "Value: " + value.name() + ", Type: " + getType() + ", part of a transition: " + isPartOfTransition();
    }

    public abstract String getType();
}
