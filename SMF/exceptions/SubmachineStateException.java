package frc.robot.ShamLib.SMF.exceptions;

import frc.robot.ShamLib.SMF.StateMachine;

public class SubmachineStateException extends Exception{

    public SubmachineStateException(String subsystemName, String state, StateMachine<?> machine, SubmachineReason r) {
        super(r.label + "\nSubsystem: " + subsystemName + "\nState: " + state + "\n" + machine);
    }

    public enum SubmachineReason {
        AlreadyFlagState("Submachine states cannot already be flag states"),
        AlreadyContinuousCommand("Submachine states cannot be continuous commands")
        ;

        private final String label;
        private SubmachineReason(String label) {
            this.label = label;
        }
    }
}