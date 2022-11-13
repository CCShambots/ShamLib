package frc.robot.ShamLib.SMF.exceptions;

import frc.robot.ShamLib.SMF.transitions.TransitionBase;

public class InvalidTransitionException extends Exception{

    public InvalidTransitionException(String subsystemName, TransitionBase<?> proposedTransition, TransitionReason reason) {
        super(reason.label + ", \n Transition details: " + proposedTransition + "\nSubsystem name: " + subsystemName);
    }

    public enum TransitionReason {
        StateAlreadyFlag("Transitions cannot include states marked as flag states"),
        DuplicateTransition("You've already definted the command to and from the two states")
        ;

        private final String label;
        private TransitionReason(String label) {
            this.label = label;
        }

    }
    
}
