package frc.robot.ShamLib.motors.rev;

public class SmartMotionValues {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kIZone;
    public final double kFF;
    public final double outputRange;
    public final double maxVel;
    public final double minVel;
    public final double maxAcc;
    public final double allowedError;

    /**
     * For use with the SparkWithAbsoluteControl class
     * @param kP P value
     * @param kI I Value
     * @param kD D value
     * @param kIZone absolute value error required for I to take effect
     * @param kFF Feedforward value
     * @param outputRange magnitude of output allowed (0, 1]
     * @param maxVel maximum controller velocity (in output units)
     * @param minVel minimum controller velocity (in output units)
     * @param maxAcc maximum controller velocity (in output units^2)
     * @param allowedError allowed error for the controller to avoid oscillations
     */
    public SmartMotionValues(double kP, double kI, double kD, double kIZone, double kFF, double outputRange, double maxVel, double minVel, double maxAcc, double allowedError) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kIZone = kIZone;
        this.kFF = kFF;
        this.outputRange = outputRange;
        this.maxVel = maxVel;
        this.minVel = minVel;
        this.maxAcc = maxAcc;
        this.allowedError = allowedError;
    }
}
