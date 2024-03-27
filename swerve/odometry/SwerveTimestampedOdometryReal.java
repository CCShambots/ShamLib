package frc.robot.ShamLib.swerve.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;
import frc.robot.ShamLib.swerve.module.SwerveModule;
import java.util.List;
import java.util.function.Supplier;

public class SwerveTimestampedOdometryReal implements SwerveOdometry {
  private final TimestampedPoseEstimator estimator;
  private final SwerveDriveKinematics kinematics;
  private final List<SwerveModule> modules;
  private final Supplier<Rotation2d> gyroAngleSupplier;

  private Rotation2d lastAngle;

  public SwerveTimestampedOdometryReal(
      TimestampedPoseEstimator estimator,
      SwerveDriveKinematics kinematics,
      List<SwerveModule> modules,
      Supplier<Rotation2d> gyroAngle) {
    this.estimator = estimator;
    this.kinematics = kinematics;
    this.modules = modules;
    this.gyroAngleSupplier = gyroAngle;

    lastAngle = gyroAngle.get();
  }

  @Override
  public void resetPose(
      Rotation2d gyroAngle, Pose2d newPose, SwerveModulePosition[] modulePositions) {
    estimator.resetPose(newPose);
  }

  @Override
  public Pose2d getPose() {
    return estimator.getLatestPose();
  }

  @Override
  public void addTimestampedVisionMeasurements(
      List<TimestampedPoseEstimator.TimestampedVisionUpdate> visionEstimates) {
    estimator.addVisionData(visionEstimates);
  }

  @Override
  public void updatePose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    updatePose();
  }

  @Override
  public void updatePose() {
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] = modules.get(i).getPositionDelta();
    }

    Twist2d twist = kinematics.toTwist2d(wheelDeltas);
    Rotation2d currentGyro = gyroAngleSupplier.get();

    twist = new Twist2d(twist.dx, twist.dy, currentGyro.minus(lastAngle).getRadians());
    lastAngle = gyroAngleSupplier.get();

    estimator.addDriveData(Timer.getFPGATimestamp(), twist);
  }
}
