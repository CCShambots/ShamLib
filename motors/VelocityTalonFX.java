package frc.robot.ShamLib.motors;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

public class VelocityTalonFX extends EnhancedTalonFX {

    private int kTimeoutMs = 30;

    private double target; //In output units

    /**
     * Constructor for a velocity configured TalonFX
     * @param deviceNumber CAN ID
     * @param canbus name of the canbus (i.e. for CANivore)
     * @param gains PIDF gains
     * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output units
     */
    public VelocityTalonFX(int deviceNumber, String canbus, PIDFGains gains, double inputToOutputRatio) {
        super(deviceNumber, canbus, inputToOutputRatio);

        configFactoryDefault();
        configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs);

        setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);

        configNominalOutputForward(0, kTimeoutMs);
        configNominalOutputReverse(0, kTimeoutMs);
        configPeakOutputForward(1, kTimeoutMs);
        configPeakOutputReverse(-1, kTimeoutMs);

        configurePIDLoop(0, gains);
    }

    /**
     * Constructor for a motion magic configured TalonFX
     * @param deviceNumber CAN ID
     * @param gains PIDF gains
     * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output units
     */
    public VelocityTalonFX(int deviceNumber, PIDFGains gains, double inputToOutputRatio) {
        this(deviceNumber, "", gains, inputToOutputRatio);
    }

    /**
     * Set the target of the motor position
     * @param target target position (in output units)
     */
    public void setTarget(double target) {
        this.target = target;

        set(TalonFXControlMode.Velocity, outputToTicks(target));
    }

    /**
     * Returns the target of the motor in output units
     * @return output units
     */
    public double getTarget() {
        return target;
    }
}
