package frc.robot.ShamLib.swerve.module;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

  @AutoLog
  public static class SwerveModuleInputs {
    public double turnEncoderPos = 0.0; // degrees

    public double turnMotorAngle = 0.0; // degrees
    public double turnMotorVelocity = 0.0; // degrees per second
    public double turnMotorTarget = 0.0; // degrees

    public double driveMotorPosition = 0.0; // meters
    public double driveMotorVelocity = 0.0; // meters per second
    public double driveMotorTarget = 0.0; // meters per second
  }

  public default void updateInputs(SwerveModuleInputs inputs) {}

  public default void applyCurrentLimit(TalonFX motor, CurrentLimitsConfigs limit) {}

  public default Command calculateTurnKV(double kS, Trigger increment, BooleanSupplier interrupt) {
    return new InstantCommand();
  }

  public default Command calculateDriveKV(
      double kS, Trigger increment, Trigger invert, BooleanSupplier interrupt, boolean telemetry) {
    return new InstantCommand();
  }

  public default void resetTurnMotorPosition(double pos) {}

  public default void setTurnMotorTarget(double target) {}

  public default void setDriveMotorTarget(double target) {}
}
