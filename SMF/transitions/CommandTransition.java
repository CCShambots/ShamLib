package frc.robot.ShamLib.SMF.transitions;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandTransition<E extends Enum<E>> extends TransitionBase<E> {
    private Command command;
    private AtomicBoolean finished = new AtomicBoolean(false);

    public CommandTransition(E startState, E endState, Command command) {
        super(startState, endState);
        this.command = command.andThen(() -> finished.set(true));
    }

    @Override
    public String toString() {
        return "Start state: " + this.startState.name() + ", End state: " + this.endState.name() + ", Command: " + this.command.toString();
    }

    public Command getCommand() {return command;}

    @Override
    public BooleanSupplier execute() {
        finished.set(false);
        command.schedule();
        return () -> finished.get();
    }
}
