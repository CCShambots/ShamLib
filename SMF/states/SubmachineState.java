package frc.robot.ShamLib.SMF.states;

import frc.robot.ShamLib.SMF.StateMachine;

public class SubmachineState<E extends Enum<E>> extends StateBase<E> {

    private StateMachine<?> machine;

    public SubmachineState(E value, StateMachine<?> machine) {
        super(value);

        this.machine = machine;
    }

    public StateMachine<?> getMachine() {return machine;}


    @Override
    public String getType() {
        return "Submachine";
    }
}
