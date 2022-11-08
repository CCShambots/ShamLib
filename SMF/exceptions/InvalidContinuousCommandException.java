package frc.robot.ShamLib.SMF.exceptions;

import edu.wpi.first.wpilibj2.command.Command;

public class InvalidContinuousCommandException extends Exception {

    public InvalidContinuousCommandException(String subsystemName, Command c, String stateName, ContinuousCommandReason r)  {
        super(r.label + "\nWith command: " + c.getName() + "\nOn state: " + stateName);
    }

    public enum ContinuousCommandReason {
        AlreadyFlagState("States cannot have continuous commands when they are already flag states"),
        CommandAlreadyDefined("States cannot have multiple continuous commands"),
        InstanceBasedState("Instance-based states cannot have continuous commands"),
        AlreadySubmachine("Continuous Commands cannot be Submachine states")
        ;

        private final String label;
        private ContinuousCommandReason(String label) {
            this.label = label;
        }
    }
    
}
