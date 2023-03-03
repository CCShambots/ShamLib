package frc.robot.ShamLib.motors.pro;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.VelocityVoltage;

public class VelocityTalonFXPro extends EnhancedTalonFXPro {

    private double target; //In output units

    /**
     * Constructor for a velocity configured TalonFX
     * @param deviceNumber CAN ID
     * @param canbus name of the canbus (i.e. for CANivore)
     * @param gains PIDF gains
     * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output units
     */
    public VelocityTalonFXPro(int deviceNumber, String canbus, PIDSVGains gains, double inputToOutputRatio) {
        super(deviceNumber, canbus, inputToOutputRatio);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0 = configurePIDLoop(gains);
        
        getConfigurator().apply(config);
    }

    /**
     * Constructor for a motion magic configured TalonFX
     * @param deviceNumber CAN ID
     * @param gains PIDF gains
     * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output units
     */
    public VelocityTalonFXPro(int deviceNumber, PIDSVGains gains, double inputToOutputRatio) {
        this(deviceNumber, "", gains, inputToOutputRatio);
    }

    /**
     * Set the target of the motor position
     * @param target target position (in output units/sec)
     */
    public void setTarget(double target) {
        setControl(new VelocityVoltage(outputToTicks(target)).withSlot(0));

        this.target = target;
    }

    /**
     * Returns the target of the motor in output units
     * @return output units
     */
    public double getTarget() {
        return target;
    }
}
