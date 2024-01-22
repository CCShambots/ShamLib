package frc.robot.ShamLib.motors.tuning;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import org.apache.commons.math3.stat.regression.SimpleRegression;

public class LinearTuningCommand extends Command {
  private final SimpleRegression regression = new SimpleRegression(true);

  private final DoubleConsumer setMotorVoltage;
  private final DoubleSupplier getMotorVoltage;
  private final DoubleSupplier getMotorVelocity;

  private final double incrementAmount;

  private int currentIncrement = 0;
  private boolean isFinished = false;

  public LinearTuningCommand(
      Trigger stop,
      Trigger incrementUp,
      Trigger incrementDown,
      DoubleConsumer setMotorVoltage,
      DoubleSupplier getMotorVelocity,
      DoubleSupplier getMotorVoltage,
      double incrementAmount) {
    this.incrementAmount = incrementAmount;

    this.setMotorVoltage = setMotorVoltage;
    this.getMotorVelocity = getMotorVelocity;
    this.getMotorVoltage = getMotorVoltage;

    stop.onTrue(new InstantCommand(() -> isFinished = true));
    incrementDown.onTrue(new InstantCommand(this::incrementDown));
    incrementUp.onTrue(new InstantCommand(this::incrementUp));
  }

  private void incrementUp() {
    if (currentIncrement * incrementAmount < 12) {
      currentIncrement++;
    }
  }

  private void incrementDown() {
    if (currentIncrement * incrementAmount > -12) {
      currentIncrement--;
    }
  }

  @Override
  public void initialize() {
    isFinished = false;
    regression.clear();
    currentIncrement = 0;
  }

  @Override
  public void execute() {
    regression.addData(getMotorVelocity.getAsDouble(), getMotorVoltage.getAsDouble());

    setMotorVoltage.accept(currentIncrement * incrementAmount);
  }

  @Override
  public void end(boolean interrupted) {
    setMotorVoltage.accept(0);

    double kV = regression.getSlope();
    double kS = regression.getIntercept();

    System.out.printf("Calculated kV -> %.4f\nCalculated kS -> %.4f", kV, kS);
  }

  @Override
  public boolean isFinished() {
    // to avoid overusing memory
    return regression.getN() > 30_000 || isFinished;
  }
}
