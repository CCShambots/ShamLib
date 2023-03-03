package frc.robot.ShamLib.motors.pro;

import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotionMagicTalonFXPro extends EnhancedTalonFXPro {
    private int kTimeoutMs = 30;

    private double target; //In output units

    /**
     * Constructor for a motion magic configured TalonFX
     * @param deviceNumber CAN ID
     * @param canbus name of the canbus (i.e. for CANivore)
     * @param gains PIDF gains
     * @param inputToOutputRatio number to multiply TalonFX rotations by to get output units
     * @param maxVel maximum velocity the motor should reach
     * @param maxAccel maximum acceleration the motor should undergo
     */
    public MotionMagicTalonFXPro(int deviceNumber, String canbus, PIDSVGains gains, double inputToOutputRatio, double maxVel, double maxAccel, double jerk) {
        super(deviceNumber, canbus, inputToOutputRatio);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0 = configurePIDLoop(gains);

        //Set the acceleration and cruise velocity - see documentation
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

        motionMagicConfigs.MotionMagicAcceleration = maxAccel;
        motionMagicConfigs.MotionMagicCruiseVelocity = maxVel;
        motionMagicConfigs.MotionMagicJerk = jerk;

        config.MotionMagic = motionMagicConfigs;

        getConfigurator().apply(config);
    }

    public MotionMagicTalonFXPro(int deviceNumber, String canbus, PIDSVGains gains, double inputToOutputRatio, double maxVel, double maxAccel){
        this(deviceNumber, canbus, gains, inputToOutputRatio, maxVel, maxAccel, 10000);
    }  
    /**
     * Constructor for a motion magic configured TalonFX
     * @param deviceNumber CAN ID
     * @param gains PIDF gains
     * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output units
     */
    public MotionMagicTalonFXPro(int deviceNumber, PIDSVGains gains, double inputToOutputRatio) {
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
    public MotionMagicTalonFXPro(int deviceNumber, PIDSVGains gains, double inputToOutputRatio, double maxVel, double maxAccel) {
        this(deviceNumber, "", gains, inputToOutputRatio, maxVel, maxAccel);
    }

    public MotionMagicTalonFXPro(int deviceNumber, PIDSVGains gains, double inputToOutputRatio, double maxVel, double maxAccel, double jerk) {
        this(deviceNumber, "", gains, inputToOutputRatio, maxVel, maxAccel, jerk);
    }

    /**
     * Constructor for a motion magic configured TalonFX
     * @param deviceNumber CAN ID
     * @param canbus name of the canbus (i.e. for CANivore)
     * @param gains PIDF gains
     * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output units
     */
    public MotionMagicTalonFXPro(int deviceNumber, String canbus, PIDSVGains gains, double inputToOutputRatio) {
        this(deviceNumber, canbus, gains, inputToOutputRatio, 0, 0);
    }

    /**
     * Set the target of the motor position
     * @param target target position (in output units)
     */
    public void setTarget(double target) {
        this.target = target;
        setControl(new MotionMagicVoltage(outputToTicks(target)).withSlot(0));
    }

    /**
     * Returns the target of the motor in output units
     * @return output units
     */
    public double getTarget() {
        return target;
    }

}
