package frc.robot.ShamLib.SMF.states;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class InstanceBasedState<E extends Enum<E>> extends StateBase<E> {
    private static Command defaultCommand = new InstantCommand();
    private Command command;

    public InstanceBasedState(E value) {
        super(value);
    }

    public Command getCommand() {
        return command;
    }

    public void clearCommand() {
        command = defaultCommand;
    }

    public void setCommand(Command command) {
        this.command = command;
    }

    @Override
    public String getType() {
        return "Instance-based";
    }
}
