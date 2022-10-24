package frc.robot.ShamLib.motors;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import edu.wpi.first.wpilibj2.command.Command;

public class MotionMagicTalonFX extends EnhancedTalonFX{
    private int kTimeoutMs = 30;

    private double target; //In output units

    /**
     * Constructor for a motion magic configured TalonFX
     * @param deviceNumber CAN ID
     * @param canbus name of the canbus (i.e. for CANivore)
     * @param gains PIDF gains
     * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output units
     * @param maxVel maximum velocity the motor should reach
     * @param maxAccel maximum acceleration the motor should undergo
     */
    public MotionMagicTalonFX(int deviceNumber, String canbus, PIDFGains gains, double inputToOutputRatio, double maxVel, double maxAccel) {
        super(deviceNumber, canbus, inputToOutputRatio);

        configFactoryDefault();
        configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs);

        setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

        configNominalOutputForward(0, kTimeoutMs);
        configNominalOutputReverse(0, kTimeoutMs);
        configPeakOutputForward(1, kTimeoutMs);
        configPeakOutputReverse(-1, kTimeoutMs);

        configurePIDLoop(0, gains);

        //Set the acceleration and cruise velocity - see documentation
        if(maxVel > 0 && maxAccel > 0) {
            configMotionCruiseVelocity(outputToTicks(maxVel) * 10, kTimeoutMs);
            configMotionAcceleration(outputToTicks(maxAccel) * 10, kTimeoutMs);
        }
    }

    /**
     * Constructor for a motion magic configured TalonFX
     * @param deviceNumber CAN ID
     * @param gains PIDF gains
     * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output units
     */
    public MotionMagicTalonFX(int deviceNumber, PIDFGains gains, double inputToOutputRatio) {
        this(deviceNumber, "", gains, inputToOutputRatio, 0, 0);
    }

    /**
     * Constructor for a motion magic configured TalonFX
     * @param deviceNumber CAN ID
     * @param gains PIDF gains
     * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output units
     * @param maxVel maximum velocity the motor should reach
     * @param maxAccel maximum acceleration the motor should underg
     */
    public MotionMagicTalonFX(int deviceNumber, PIDFGains gains, double inputToOutputRatio, double maxVel, double maxAccel) {
        this(deviceNumber, "", gains, inputToOutputRatio, maxVel, maxAccel);
    }

    /**
     * Constructor for a motion magic configured TalonFX
     * @param deviceNumber CAN ID
     * @param canbus name of the canbus (i.e. for CANivore)
     * @param gains PIDF gains
     * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output units
     */
    public MotionMagicTalonFX(int deviceNumber, String canbus, PIDFGains gains, double inputToOutputRatio) {
        this(deviceNumber, canbus, gains, inputToOutputRatio, 0, 0);
    }

    /**
     * Set the target of the motor position
     * @param target target position (in output units)
     */
    public void setTarget(double target) {
        this.target = target;

        set(TalonFXControlMode.MotionMagic, outputToTicks(target));
    }

    /**
     * Returns the target of the motor in output units
     * @return output units
     */
    public double getTarget() {
        return target;
    }

}
