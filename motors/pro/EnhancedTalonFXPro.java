package frc.robot.ShamLib.motors.pro;

import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

public class EnhancedTalonFXPro extends TalonFX {
    private int kTimeoutMs = 30;

    private double inputToOutputRatio;
    private double target; //In output units

    /**
     * Constructor for a TalonFX with gear ratio math included
     * @param deviceNumber CAN ID
     * @param canbus name of the canbus (i.e. for CANivore)
     * @param inputToOutputRatio number to multiply TalonFX rotations by to get output units (i.e. degree/tick)
     */
    public EnhancedTalonFXPro(int deviceNumber, String canbus, double inputToOutputRatio) {
        super(deviceNumber, canbus);

        this.inputToOutputRatio = inputToOutputRatio;

    }

    /**
     * Constructor for a TalonFX with gear ratio math included
     * @param deviceNumber CAN ID
     * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output units
     */
    public EnhancedTalonFXPro(int deviceNumber, double inputToOutputRatio) {
        this(deviceNumber, "", inputToOutputRatio);
    }

    /**
     * Get the position of the encoder (in output units)
     * @return output units
     */
    public double getEncoderPosition() {
        return ticksToOutput(getPosition().getValue());
    }


    /**
     * Get the velocity of the motor (in output units / second)
     * @return output units / sec
     */
    public double getEncoderVelocity() {
        return ticksToOutput(getRotorVelocity().getValue());
    }


    /**
     * Set a percentage power to the motor
     * @param power decimal from -1 to 1 to set the motor's pwoer to
     */
    public void setManualPower(double power) {
        set(power);
    }

    /**
     * Converts some number of encoder ticks into output units
     * @param ticks encoder ticks
     * @return output units
     */
    public double ticksToOutput(double ticks) {
        return ticks * inputToOutputRatio;
    }

    /**
     * Converts some number of output units into encoder ticks
     * @param output output units
     * @return encoder ticks
     */
    public double outputToTicks(double output) {
        return output / inputToOutputRatio;
    }

    /**
     * @param pos The position (in output units) to which the motor should be reset
     */
    public void resetPosition(double pos) {
        setRotorPosition(outputToTicks(pos));
    }

    /**
     * Resets the motor back to zero
     */
    public void resetPosition() {
        resetPosition(0);
    }

    /**
     * Configure the motors PIDF loop for motion magic
     * @param gains PIDF gains
     */
    public Slot0Configs configurePIDLoop(PIDSVGains gains) {
        //Set the motion magic gains in slot0
        Slot0Configs pidConfigs = new Slot0Configs();
        pidConfigs.kS = gains.getS();
        pidConfigs.kV = gains.getV();
        pidConfigs.kP = gains.getP();
        pidConfigs.kI = gains.getI();
        pidConfigs.kD = gains.getD();

        return pidConfigs;
    }


    //TODO: Make this not garbage
    /**
     * @param kS kS value on the motor
     * @param voltageIncrement amount of volts to increment per tap
     * @param increment trigger to increase amount of voltage
     * @param interrupt supplier to interrupt 
     * @return the command to run
     */
    public Command calculateKV(double kS, double voltageIncrement, Trigger increment, BooleanSupplier interrupt, boolean telemetry) {

        AtomicInteger currentMultiple = new AtomicInteger();

        increment.onTrue(new InstantCommand(currentMultiple::getAndIncrement));

        return new FunctionalCommand(
                () -> {
                    System.out.println("Starting KV calculation");
                    currentMultiple.set(0);
                },
                () -> {
                    double voltage = kS + currentMultiple.get() * voltageIncrement;

                    setVoltage(kS + currentMultiple.get() * voltageIncrement);

                    if(telemetry) System.out.println("(KV) volts: " + (voltage-kS) + ", velocity: " + getVelocity().getValue());

                },
                (interrupted) -> {
                    setManualPower(0);
                },
                () -> interrupt.getAsBoolean()
        );
    }

    public Command calculateKV(double kS, double voltageIncrement, Trigger increment, BooleanSupplier interrupt) {
        return calculateKV(kS, voltageIncrement, increment, interrupt, true);
    }

}
