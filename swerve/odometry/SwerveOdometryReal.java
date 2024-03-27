package frc.robot.ShamLib.swerve.odometry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;
import java.util.List;

public class SwerveOdometryReal implements SwerveOdometry {

  private final SwerveDrivePoseEstimator odometry;

  public SwerveOdometryReal(SwerveDrivePoseEstimator odometry) {
    this.odometry = odometry;
  }

  @Override
  public void addVisionMeasurement(Pose2d pose) {
    odometry.addVisionMeasurement(getPose(), Timer.getFPGATimestamp());
  }

  @Override
  public void addTimestampedVisionMeasurements(
      List<TimestampedPoseEstimator.TimestampedVisionUpdate> visionEstimates) {
    for (var visionEstimate : visionEstimates) {
      odometry.addVisionMeasurement(
          visionEstimate.pose(), visionEstimate.timestamp(), visionEstimate.stdDevs());
    }
  }

  @Override
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  @Override
  public void resetPose(
      Rotation2d gyroAngle, Pose2d newPose, SwerveModulePosition[] modulePositions) {
    odometry.resetPosition(gyroAngle, modulePositions, newPose);
  }

  @Override
  public void updatePose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    odometry.update(gyroAngle, modulePositions);
  }
}
