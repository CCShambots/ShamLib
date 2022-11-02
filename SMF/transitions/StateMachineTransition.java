package frc.robot.ShamLib.SMF.transitions;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShamLib.SMF.StateMachine;

public class StateMachineTransition<E extends Enum<E>> extends TransitionBase<E> {
    private StateMachine<?> machine;
    private AtomicBoolean finished = new AtomicBoolean(false);

    public StateMachineTransition(E startState, E endState, StateMachine<?> machine) {
        super(startState, endState);
        this.machine = machine;
    }

    @Override
    public String toString() {
        return "Start state: " + this.startState.name() + ", End state: " + this.endState.name() + ", State machine: " + this.machine.getName();
    }

    public StateMachine<?> getMachine() {return machine;}

    @Override
    public BooleanSupplier execute() {
        finished.set(false);
        machine.schedule();
        return () -> finished.get();
    }

    @Override
    public void cancel() {
        command.cancel();        
    }
}
