package frc.robot.ShamLib;

import edu.wpi.first.math.controller.PIDController;

public class PIDGains {
    public final double p;
    public final double i;
    public final double d;

    public PIDGains(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public PIDController applyToController() {
        return new PIDController(p, i, d);
    }
}
