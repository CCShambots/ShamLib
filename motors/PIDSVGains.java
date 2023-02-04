package frc.robot.ShamLib.motors;

public class PIDSVGains {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;
    public final double kS;
    public final double kV;

    public PIDSVGains(double kP, double kI, double kD, double kF, double kS, double kV) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.kS = kS;
        this.kV = kV;
    }
}
