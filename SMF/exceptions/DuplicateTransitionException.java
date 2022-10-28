package frc.robot.ShamLib.SMF.exceptions;

import frc.robot.ShamLib.SMF.transitions.TransitionBase;

public class DuplicateTransitionException extends Exception{

    public DuplicateTransitionException(String subsystemName, TransitionBase<?> t1, TransitionBase<?> t2) {
        super("Transition: " + t1 + "\nis not compatible with " + t2 + "\n in subsystem: " + subsystemName);
    }
    
}
