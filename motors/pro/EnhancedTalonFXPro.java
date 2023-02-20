package frc.robot.ShamLib.motors.pro;

import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;
import java.util.List;
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
        pidConfigs.kS = gains.kS;
        pidConfigs.kV = gains.kV;
        pidConfigs.kP = gains.kP;
        pidConfigs.kI = gains.kI;
        pidConfigs.kD = gains.kD;

        return pidConfigs;
    }

    /**
     * Command to calculate the kF value for a PIDF loop to run on a motor
     * It will run until the interrupt is triggered, and then print its results to the console
     * @param power raw power of the motor (0-1)
     * @param offsetTime time to wait before tracking data (seconds)
     * @param interrupt the condition to end the command
     * @return the command to run
     */
    public Command calculateKV(double power, double offsetTime, BooleanSupplier interrupt) {
        List<Double> rawVelos = new ArrayList<>();
        List<Double> filteredVelos = new ArrayList<>();
        LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
        
        Timer timer = new Timer();

        return new FunctionalCommand(
                () -> {
                    setManualPower(Math.abs(power));
                    timer.start();
                },
                () -> {
                    if(timer.get() > offsetTime) {
                        filteredVelos.add(filter.calculate(getVelocity().getValue()));
                        rawVelos.add(getVelocity().getValue());
                    }
                },
                (interrupted) -> {
                    setManualPower(0);
                    double maxFiltered = filteredVelos.stream().max(Double::compare).get();
                    double maxRaw = rawVelos.stream().max(Double::compare).get();
                    System.out.println("filtered kF: " + (power * 1023.0) / maxFiltered);
                    System.out.println("raw kF: " + (power * 1023.0) / maxRaw);
                },
                () -> interrupt.getAsBoolean()
        );
    }

    public Command calculateKV(double power, BooleanSupplier interrupt) {
        return calculateKV(power, 1, interrupt);
    }

    public Command calculateKV(double power) {
        return calculateKV(power, 1, () -> false);
    }

    public Command calculateKS(Trigger incrementPower, double voltageIncrement) {
        AtomicReference<Double> volts = new AtomicReference<>((double) 0);

        incrementPower.onTrue(new InstantCommand(() -> volts.set(volts.get() + voltageIncrement)));

        return new FunctionalCommand(
            () -> {

            },
            () -> setVoltage(volts.get()),
            (interrupted) -> {
                System.out.println("Volts: " + volts.get() + ", Velo (units/sec): " + getEncoderVelocity());
            },
            () -> getEncoderVelocity() > (1.0/60.0)
        );
    }


    public Command calculateKS(Trigger incrementPower) {
        return calculateKS(incrementPower, 0.05);
    }
}

