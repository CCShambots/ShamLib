package frc.robot.util.Shambots5907_SMF;

import java.util.function.BooleanSupplier;

public class FlagState<E extends Enum<E>> {
    private E state;
    private BooleanSupplier condition;

    public FlagState(E state, BooleanSupplier condition) {
        this.state = state;
        this.condition = condition;
    }

    public E getState() {
        return state;
    }

    public BooleanSupplier getCondition() {
        return condition;
    }
}
