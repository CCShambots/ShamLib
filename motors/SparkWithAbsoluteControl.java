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
     *                           (in radians) to output radians
     */
    public SparkWithAbsoluteControl(int deviceId, MotorType type, SmartMotionValues gains, double encoderOffset, double inputToOutputRatio) {
        super(deviceId, type);

        restoreFactoryDefaults();

        absoluteEncoder = getAbsoluteEncoder(kDutyCycle);
        this.encoderOffset = encoderOffset;

        syncRelativePosition();

        SparkMaxPIDController controller = getPIDController();

        controller.setFeedbackDevice(absoluteEncoder);
        controller.setP(gains.kP);
        controller.setI(gains.kI);
        controller.setD(gains.kD);
        controller.setIZone(gains.kIZone);


        controller.setSmartMotionAllowedClosedLoopError()
    }

    /**
     * Get the position of the absolute encoder
     * @return encoder pos (radians)
     */
    public double getAbsolutePosition() {
        return Math.IEEEremainder(absoluteEncoder.getPosition() * 2 * PI - encoderOffset, 2 * PI);
    }

    public double getRelativeEncoderPosition() {
        return getEncoder().getPosition() * 2 * PI * inputToOutputRatio;
    }

    public void setRelativeEncoderPosition(double outputRadians) {
        getEncoder().setPosition((outputRadians / inputToOutputRatio) / (2*PI));
    }

    /**
     * Syncs the relative encoder position to the position of the absolute encoder
     */
    public void syncRelativePosition() {
        setRelativeEncoderPosition(getAbsolutePosition());
    }


}
