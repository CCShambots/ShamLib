package frc.robot.ShamLib.motors.rev;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import static com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle;
import static java.lang.Math.PI;

public class PositionSpark extends CANSparkMax implements Sendable {

    private final AbsoluteEncoder absoluteEncoder;

    // private ProfiledPIDController localController;

    private final double encoderOffset;
    private final SparkMaxPIDController controller;
    private double target = 0;
    private final double tolerance;

    /**
     * Create a new object to control a SPARK MAX motor Controller
     *
     * @param deviceId The device ID.
     * @param type     The motor type connected to the controller. Brushless motor wires must be connected
     *                 to their matching colors and the hall sensor must be plugged in. Brushed motors must be
     *                 connected to the Red and Black terminals only.
     * @param gains the PIDF gains to apply to the motor's velocity control
     * @param encoderOffset the offset of the absolute encoder from the desired zero position (in radians)
     */
    public PositionSpark(int deviceId, MotorType type, PIDFGains gains, double encoderOffset, double tolerance) {
        super(deviceId, type);

        restoreFactoryDefaults();

        absoluteEncoder = getAbsoluteEncoder(kDutyCycle);
        this.encoderOffset = encoderOffset;

        absoluteEncoder.setZeroOffset(encoderOffset / (2 * PI) + 0.5 );

        controller = getPIDController();

        controller.setFeedbackDevice(absoluteEncoder);
        controller.setP(gains.getP());
        controller.setI(gains.getI());
        controller.setD(gains.getD());

        controller.setIZone(0.05);

        controller.setFF(gains.getF());

        controller.setOutputRange(-0.5, 0.5);

        this.tolerance = tolerance;

    }

    public void update() {
        // if(Math.abs(getTarget() - getPosition()) <= tolerance) set(0);
    }

    public void setTarget(double target) {
        this.target = target;

        controller.setReference(target / (2 * PI) + 0.5, ControlType.kPosition);
    }

    public double getTarget() {
        return target;
    }

    /**
     * Get the position of the absolute encoder (in radians)
     * @return the position
     */
    public double getPosition() {
        return (absoluteEncoder.getPosition() - 0.5) * 2 * PI;
    }

    /**
     * Get the current velocity of the encoder
     * @return the velocity
     */
    public double getVelocity() {
        return getEncoder().getVelocity() * 2 * PI;
    }

    public double getMotorOutputPower() {
        return getAppliedOutput();
    }

    // public double getPositionError() {
        // return abs(getPosition() - localController.getSetpoint().position);
    // }

    // public double getVeloError() {
        // return abs(getVelocity() - localController.getSetpoint().velocity);
    // }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("output-power", () -> getMotorOutputPower(), null);
        builder.addDoubleProperty("position", () -> getPosition(), null);
        builder.addDoubleProperty("target", () -> getTarget(), null);
        // builder.addDoubleProperty("position-error", () -> getPositionError(), null);
        // builder.addDoubleProperty("velocity-error", () -> getVeloError(), null);
    }
}
