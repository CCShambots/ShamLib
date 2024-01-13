package frc.robot.ShamLib.motors.tuning;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.apache.commons.math3.stat.regression.SimpleRegression;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class LinearTuningCommand extends Command {
    private final SimpleRegression regression = new SimpleRegression(true);

    private final DoubleConsumer setMotorVoltage;
    private final DoubleSupplier getMotorVoltage;
    private final DoubleSupplier getMotorVelocity;

    private final double incrementAmount;

    private int currentIncrement = 0;

    public LinearTuningCommand(
            Trigger stop,
            Trigger incrementUp,
            Trigger incrementDown,
            DoubleConsumer setMotorVoltage,
            DoubleSupplier getMotorVelocity,
            DoubleSupplier getMotorVoltage,
            double incrementAmount
    ) {
        this.incrementAmount = incrementAmount;

        this.setMotorVoltage = setMotorVoltage;
        this.getMotorVelocity = getMotorVelocity;
        this.getMotorVoltage = getMotorVoltage;

        stop.onTrue(new InstantCommand(this::cancel));
        incrementDown.onTrue(new InstantCommand(this::incrementDown));
        incrementUp.onTrue(new InstantCommand(this::incrementUp));
    }

    private void incrementUp() {
        currentIncrement++;
    }

    private void incrementDown() {
        currentIncrement--;
    }

    @Override
    public void initialize() {
        regression.clear();
        currentIncrement = 0;
    }

    @Override
    public void execute() {
        regression.addData(getMotorVoltage.getAsDouble(), getMotorVelocity.getAsDouble());

        setMotorVoltage.accept(currentIncrement * incrementAmount);
    }

    @Override
    public void end(boolean interrupted) {
        setMotorVoltage.accept(0);

        double kV = regression.getSlope();
        double kS = regression.getIntercept();

        System.out.printf("Calculated kV -> %.4f\nCalculated kS -> %.4f%n", kV, kS);
    }

    @Override
    public boolean isFinished() {
        //to avoid exploding
        return regression.getN() > 30_000;
    }
}
