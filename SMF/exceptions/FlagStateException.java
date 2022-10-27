package frc.robot.ShamLib.SMF.exceptions;

public class FlagStateException extends Exception{
    public FlagStateException(String subsystemName, String flagState, String parentState, FlagStateReason r) {
        super(r.label + "\nSubsystem: " + subsystemName + "\nFlag state: " + flagState + "\nParent state: " + parentState);
    }

    public enum FlagStateReason {
        PartOfTransition("Flag states cannot be part of an existing transition"),
        FlagAlreadyParent("Flag states cannot already be marked as parent states"),
        ParentAlreadyFlag("Parent states cannot already be flag states")
        ;

        private final String label;
        private FlagStateReason(String label) {
            this.label = label;
        }
    }
}