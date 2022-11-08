package frc.robot.ShamLib.SMF.states;

public abstract class StateBase<E extends Enum<E>> {
    private E value;

    public StateBase(E value) {
        this.value = value;
    }

    public E getValue() {
        return value;
    }

    @Override
    public String toString() {
        return value.name();
    }

    public abstract String getType();
}
