package frc.robot.ShamLib.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.BooleanSupplier;

public class EnhancedTalonFX extends WPI_TalonFX {
    private int kTimeoutMs = 30;

    private double inputToOutputRatio;
    private double target; //In output units

    /**
     * Constructor for a TalonFX with gear ratio math included
     * @param deviceNumber CAN ID
     * @param canbus name of the canbus (i.e. for CANivore)
     * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output units
     */
    public EnhancedTalonFX(int deviceNumber, String canbus, double inputToOutputRatio) {
        super(deviceNumber, canbus);

        this.inputToOutputRatio = inputToOutputRatio;

    }

    /**
     * Constructor for a TalonFX with gear ratio math included
     * @param deviceNumber CAN ID
     * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output units
     */
    public EnhancedTalonFX(int deviceNumber,double inputToOutputRatio) {
        this(deviceNumber, "", inputToOutputRatio);
    }

    /**
     * Get the position of the encoder (in output units)
     * @return output units
     */
    public double getPosition() {
        return ticksToOutput(getSelectedSensorPosition());
    }

    /**
     * Get the velocity of the motor (in output units / second)
     * @return output units / sec
     */
    public double getVelocity() {
        return ticksToOutput(getSelectedSensorVelocity()) * 10;
    }


    /**
     * Set a percentage power to the motor
     * @param power decimal from -1 to 1 to set the motor's pwoer to
     */
    public void setManualPower(double power) {
        set(ControlMode.PercentOutput, power);
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
     * Returns the target of the motor in native encoder ticks
     * @return encoder ticks
     */
    public double getTicksTarget() {
        return getClosedLoopTarget();
    }

    /**
     * @param pos The position (in output units) to which the motor should be reset
     */
    public void resetPosition(double pos) {
        setSelectedSensorPosition(outputToTicks(pos));
    }

    /**
     * Resets the motor back to zero
     */
    public void resetPosition() {
        setSelectedSensorPosition(0);
    }

    /**
     * Configure the motors PIDF loop for motion magic
     * @param idx id slot of the PID
     * @param gains PIDF gains
     */
    public void configurePIDLoop(int idx, PIDFGains gains) {
        //Set the motion magic gains in slot0
        selectProfileSlot(idx, 0);
        config_kF(idx, gains.kF, kTimeoutMs);
        config_kP(idx, gains.kP, kTimeoutMs);
        config_kI(idx, gains.kI, kTimeoutMs);
        config_kD(idx, gains.kD, kTimeoutMs);
    }

    public Command calculateKF(double power, BooleanSupplier interrupt) {
        List<Double> rawVelos = new ArrayList<>();
        List<Double> filteredVelos = new ArrayList<>();
        LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02); //TODO: These values might have to change if they're not useful

        return new FunctionalCommand(
                () -> setManualPower(power),
                () -> {
                    filteredVelos.add(filter.calculate(getSelectedSensorVelocity()));
                    rawVelos.add(getSelectedSensorVelocity());
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
}

