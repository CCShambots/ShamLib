package frc.robot.ShamLib.swerve.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOReal implements GyroIO {
  private final Pigeon2 gyro;

  public GyroIOReal(int pigeon2ID, String gyroCanbus) {
    gyro = new Pigeon2(pigeon2ID, gyroCanbus);

    Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
    gyro.getConfigurator().apply(pigeonConfig);
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.gyroRoll = Rotation2d.fromDegrees(gyro.getRoll().getValue());
    inputs.gyroPitch = Rotation2d.fromDegrees(gyro.getPitch().getValue());
    inputs.gyroYaw = Rotation2d.fromDegrees(gyro.getYaw().getValue());
  }

  @Override
  public void setGyroYaw(Rotation2d val) {
    gyro.setYaw(val.getDegrees());
  }
}
