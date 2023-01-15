package frc.robot.ShamLib.sensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

import static java.lang.Math.PI;

public class ThroughBoreEncoder {
    private DutyCycle cycle;
    private double offset;

    public ThroughBoreEncoder(int port, double offsetDegrees) {
        this.cycle = new DutyCycle(new DigitalInput(port));
        this.offset = offsetDegrees / 360.0;
    }

    public double getRaw() {
        return Math.IEEEremainder(cycle.getOutput() - offset, 1);
    }

    public double getDegrees() {
        return getRaw() * 360;
    }

    public double getRadians() {
        return getRaw() * PI * 2;
    }
}