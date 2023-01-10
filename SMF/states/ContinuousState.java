package frc.robot.ShamLib.SMF.states;

import edu.wpi.first.wpilibj2.command.Command;

public class ContinuousState<E extends Enum<E>> extends StateBase<E> {

    private Command command;

    public ContinuousState(E value, Command command) {
        super(value);

        this.command = command;
    }

    public Command getCommand() {
        return command;
    }

    @Override
    public String getType() {
        return "Continuous state";
    }
}
