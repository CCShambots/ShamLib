package frc.robot.ShamLib.motors.pro;

public class PIDSVGains {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kS;
    public final double kV;

    public PIDSVGains(double kP, double kI, double kD, double kS, double kV) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
    }
}
