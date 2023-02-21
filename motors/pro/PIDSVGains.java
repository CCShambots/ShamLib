package frc.robot.ShamLib.motors.pro;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PIDSVGains implements Sendable {
    private double P;
    private double I;
    private double D;
    private double S;
    private double V;

    public PIDSVGains(double kP, double kI, double kD, double kS, double kV) {
        this.P = kP;
        this.I = kI;
        this.D = kD;
        this.S = kS;
        this.V = kV;
    }

    public double getP() {
        return P;
    }

    public double getI() {
        return I;
    }

    public double getD() {
        return D;
    }

    public double getS() {
        return S;
    }

    public double getV() {
        return V;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("P", () -> P, (value) -> P = value);
        builder.addDoubleProperty("I", () -> I, (value) -> I = value);
        builder.addDoubleProperty("D", () -> D, (value) -> D = value);
        builder.addDoubleProperty("S", () -> S, (value) -> S = value);
        builder.addDoubleProperty("V", () -> V, (value) -> V = value);
    }
}
