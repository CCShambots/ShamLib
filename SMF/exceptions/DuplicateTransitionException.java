package frc.robot.ShamLib.SMF.exceptions;

import frc.robot.ShamLib.SMF.Transition;

public class DuplicateTransitionException extends Exception{

    public DuplicateTransitionException(String subsystemName, Transition<?> t1, Transition<?> t2) {
        super("Transition: " + t1 + "\nis not compatible with " + t2 + "\n in subsystem: " + subsystemName);
    }
    
}
