package frc.robot.ShamLib.sensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

import static java.lang.Math.PI;

public class ThroughBoreEncoder {
    private final DutyCycle cycle;
    private final double offset;
    private boolean invert = false;

    public ThroughBoreEncoder(int port, double offsetDegrees) {
        this.cycle = new DutyCycle(new DigitalInput(port));
        this.offset = offsetDegrees / 360.0;
    }

    public void setInverted(boolean value) {
        invert = value;
    }

    public double getRaw() {
        return Math.IEEEremainder(((invert ? -1 : 1) * cycle.getOutput()) - offset, 1);
    }

    public double getDegrees() {
        return getRaw() * 360;
    }

    public double getRadians() {
        return getRaw() * PI * 2;
    }
}
