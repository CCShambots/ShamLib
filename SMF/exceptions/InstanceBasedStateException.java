package frc.robot.ShamLib.SMF.exceptions;

public class InstanceBasedStateException extends Exception{
    
    public InstanceBasedStateException(String subsystemName, String stateName, InstanceBasedReason r) {
        super(r.label + "\nSubsystem name: " + subsystemName + "\nState: " + stateName);
    }

    public enum InstanceBasedReason {
        AlreadyFlagState("Instance-based states cannot also be flag states"),
        AlreadyContinuous("Instance-based states cannot also have continuous commands")
        ;

        private final String label;
        private InstanceBasedReason(String label) {
            this.label = label;
        }
    }
    
}
