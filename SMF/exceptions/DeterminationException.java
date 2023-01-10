package frc.robot.ShamLib.SMF.exceptions;

public class DeterminationException extends Exception {
    public DeterminationException(String subsystemName, String undeterminedState, String entryState, DeterminationReason reason) {
        super(reason.label + "\nSubsysten name: " + subsystemName + "\nState designated as undetermined: " + undeterminedState + "\nState designated for entry:" + entryState);
    }

    public enum DeterminationReason {
        AlreadyExisting("You cannot define a determination for this subsystem because it already has")
        ;

        private final String label;
        private DeterminationReason(String label) {
            this.label = label;
        }
    }

    
}
