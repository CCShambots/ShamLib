package frc.robot.ShamLib.SMF.states;


public class InstanceBasedState<E extends Enum<E>> extends StateBase<E> {

    public InstanceBasedState(E value) {
        super(value);
    }

    @Override
    public String getType() {
        return "Instance-based";
    }
}
