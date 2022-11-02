package frc.robot.ShamLib.SMF.exceptions;

public class TransitionException extends Exception{
    public TransitionException(String subsystemName, String targetState, TransitionExceptionReason r) {

        super(r.label + "\nSubsytem: " + subsystemName + "\nTarget state: " + targetState);

    }

    public enum TransitionExceptionReason {
        MissingEndState("You requested a transition to a state that isn't in the state machine"),
        NotInstanceState("You tried to provide a command for an instance-based state, but the state is not instance based")
        ;

        private final String label;
        private TransitionExceptionReason(String label) {
            this.label = label;
        }
    }
}
