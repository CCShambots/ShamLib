package frc.robot.ShamLib;

public class ShamLibConstants {

    public static class SMF {
        public static double transitionTimeout = 2; //seconds
    }

    public static class Swerve {
        //How far the module can be off from the actual position to trigger a correction
        public static final double ALLOWED_MODULE_ERROR = 2; //Deg
        //How close the modules must stay to reset the modules successfully
        public static final double ALLOWED_STOPPED_MODULE_DIFF = .2; //degrees
    }
}
