package frc.robot.ShamLib.SMF.exceptions;

import frc.robot.ShamLib.SMF.Transition;

public class InvalidTransitionException extends Exception{

    public InvalidTransitionException(String subsystemName, Transition<?> proposedTransition, TransitionReason reason) {
        super(reason.label + ", \n Transition details: " + proposedTransition + "\nSubsystem name: " + subsystemName);
    }

    public enum TransitionReason {
        CommandRequirements("Commands must have no requirements or only require this subsystem!"),
        StateAlreadyFlag("Transitions cannot include states marked as flag states"),
        UndeterminedState("Transitions cannot go to or from the undetermined state")
        ;

        private final String label;
        private TransitionReason(String label) {
            this.label = label;
        }

    }
    
}
