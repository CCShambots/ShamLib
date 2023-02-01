package frc.robot.ShamLib.motors;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import static com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle;
import static java.lang.Math.PI;

public class SparkWithAbsoluteControl extends CANSparkMax {

    private AbsoluteEncoder absoluteEncoder;
    private double encoderOffset;
    private double inputToOutputRatio;
    private SparkMaxPIDController controller;
    private double target = 0;

    /**
     * Create a new object to control a SPARK MAX motor Controller
     *
     * @param deviceId The device ID.
     * @param type     The motor type connected to the controller. Brushless motor wires must be connected
     *                 to their matching colors and the hall sensor must be plugged in. Brushed motors must be
     *                 connected to the Red and Black terminals only.
     * @param gains the PIDF gains to apply to the motor's absolute position control
     * @param encoderOffset the offset of the absolute encoder from the desired zero position (in radians)
     * @param inputToOutputRatio the gear ratio of the motor, such that multiplying position of the relative encoder
     *                           (in rotation) to output units
     */
    public SparkWithAbsoluteControl(int deviceId, MotorType type, SmartMotionValues gains, double encoderOffset, double inputToOutputRatio) {
        super(deviceId, type);

        restoreFactoryDefaults();

        absoluteEncoder = getAbsoluteEncoder(kDutyCycle);
        this.encoderOffset = encoderOffset;

        absoluteEncoder.setPositionConversionFactor(inputToOutputRatio);
        absoluteEncoder.setZeroOffset(encoderOffset);

        controller = getPIDController();

        controller.setFeedbackDevice(absoluteEncoder);
        controller.setP(gains.kP);
        controller.setI(gains.kI);
        controller.setD(gains.kD);
        controller.setIZone(gains.kIZone);

        int smartMotionSlot = 0;
        controller.setSmartMotionMaxVelocity(gains.maxVel, smartMotionSlot);
        controller.setSmartMotionMinOutputVelocity(gains.minVel, smartMotionSlot);
        controller.setSmartMotionMaxAccel(gains.maxAcc, smartMotionSlot);
        controller.setSmartMotionAllowedClosedLoopError(gains.allowedError, smartMotionSlot);
    }

    public void setTarget(double target) {
        this.target = target;
        controller.setReference(target, ControlType.kSmartMotion);
    }

    public double getTarget() {
        return target;
    }

    public double getPosition() {
        return absoluteEncoder.getPosition();
    }



}
