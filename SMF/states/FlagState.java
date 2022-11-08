package frc.robot.ShamLib.SMF.states;

import java.util.function.BooleanSupplier;

public class FlagState<E extends Enum<E>> extends StateBase<E>{
    private BooleanSupplier supplier;
    private E parentState;

    public FlagState(E value, E parentState, BooleanSupplier supplier) {
        super(value);

        this.supplier = supplier;
        this.parentState = parentState;
    }

    public BooleanSupplier getSuppler() {return supplier;}
    public E getParentState() {return parentState;}

    @Override
    public String getType() {
        return "Flag state";
    }
}
