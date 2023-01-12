package frc.robot.ShamLib.SMF.exceptions;

import frc.robot.ShamLib.SMF.StateMachine;

public class IncompatibleStateException extends Exception{
    public IncompatibleStateException(StateMachine<?> machine, Object existing, Object newType) {
        super("Incompatible state conversion in " + machine.getName() + " attempted. Details: \nExisting:" + existing + "\nNew:" + newType);
    }
}
