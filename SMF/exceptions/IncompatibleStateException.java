package frc.robot.ShamLib.SMF.exceptions;

import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.SMF.states.StateBase;

public class IncompatibleStateException extends Exception{
    public IncompatibleStateException(StateMachine<?> machine, StateBase<?> existing, StateBase<?> newType) {
        super("Incompatible state conversion in " + machine.getName() + " attempted. Details: \nExisting:" + existing + "\nNew:" + newType);
    }
}
