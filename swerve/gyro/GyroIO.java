package frc.robot.ShamLib.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

  @AutoLog
  public static class GyroInputs {
    public Rotation2d gyroYaw = new Rotation2d();
    public Rotation2d gyroPitch = new Rotation2d();
    public Rotation2d gyroRoll = new Rotation2d();
  }

  public default void updateInputs(GyroInputs inputs) {}

  public default void setGyroYaw(Rotation2d val) {}
}
