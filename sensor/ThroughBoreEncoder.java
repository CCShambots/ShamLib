package frc.robot.ShamLib.sensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

import static java.lang.Math.PI;

public class ThroughBoreEncoder {
    private DutyCycle cycle;

    public ThroughBoreEncoder(int port) {
        this.cycle = new DutyCycle(new DigitalInput(port));
    }

    public double getRaw() {
        return Math.IEEEremainder(cycle.getOutput(), 1);
    }

    public double getDegrees() {
        return getRaw() * 360;
    }

    public double getRadians() {
        return getRaw() * PI * 2;
    }
}
