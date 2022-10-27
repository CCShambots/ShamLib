package frc.robot.ShamLib.SMF.exceptions;

public class TransitionException extends Exception{
    public TransitionException(String subsystemName, String targetState, TransitionExceptionReason r) {

        super(r.label + "\nSubsytem: " + subsystemName + "\nTarget state: " + targetState);

    }

    public enum TransitionExceptionReason {
        MissingEndState("You requested a transition to a state that isn't in the state machine")
        ;

        private final String label;
        private TransitionExceptionReason(String label) {
            this.label = label;
        }
    }
}
