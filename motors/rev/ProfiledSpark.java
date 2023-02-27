package frc.robot.ShamLib.motors.rev;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.motors.v5.PIDFGains;

import static com.revrobotics.CANSparkMax.ControlType.kVelocity;
import static com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

public class ProfiledSpark extends CANSparkMax implements Sendable {

    private AbsoluteEncoder absoluteEncoder;

    private ProfiledPIDController localController;

    private double encoderOffset;
    private SparkMaxPIDController controller;
    private double target = 0;

    /**
     * Create a new object to control a SPARK MAX motor Controller
     *
     * @param deviceId The device ID.
     * @param type     The motor type connected to the controller. Brushless motor wires must be connected
     *                 to their matching colors and the hall sensor must be plugged in. Brushed motors must be
     *                 connected to the Red and Black terminals only.
     * @param gains the PIDF gains to apply to the motor's velocity control
     * @param localGains gains for the profiled controller to run on the rio
     * @param maxVel in radians
     * @param maxAccel in radians
     * @param encoderOffset the offset of the absolute encoder from the desired zero position (in radians)
     */
    public ProfiledSpark(int deviceId, MotorType type, PIDFGains gains, PIDGains localGains, double maxVel, double maxAccel, double encoderOffset) {
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

        controller.setOutputRange(-1, 1);

        //Instantiate the rio controller
        this.localController = new ProfiledPIDController(localGains.p, localGains.i, localGains.d,
                new Constraints(maxVel, maxAccel));

        localController.setTolerance(Math.toDegrees(0.5)); //Radians

    }

    public void update() {
        controller.setReference(localController.calculate(getPosition()), kVelocity);
    }

    public void resetPID() {
        localController.reset(getPosition(), getVelocity());
    }

    public void setTarget(double target) {
        this.target = target;

        localController.setGoal(target);
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

    public double getPositionError() {
        return abs(getPosition() - localController.getSetpoint().position);
    }

    public double getVeloError() {
        return abs(getVelocity() - localController.getSetpoint().velocity);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("output-power", () -> getMotorOutputPower(), null);
        builder.addDoubleProperty("position", () -> getPosition(), null);
        builder.addDoubleProperty("target", () -> getTarget(), null);
        builder.addDoubleProperty("position-error", () -> getPositionError(), null);
        builder.addDoubleProperty("velocity-error", () -> getVeloError(), null);
    }
}
