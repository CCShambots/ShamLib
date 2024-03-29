package frc.robot.ShamLib.swerve.odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;
import java.util.List;

public interface SwerveOdometry {
  public static record TimestampedVisionUpdate(
      double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}

  public void resetPose(
      Rotation2d gyroAngle, Pose2d newPose, SwerveModulePosition[] modulePositions);

  public Pose2d getPose();

  public default void updatePose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {}

  public default void updatePose() {}

  public default void addVisionMeasurement(Pose2d pose) {}

  public default void addTimestampedVisionMeasurements(
      List<TimestampedPoseEstimator.TimestampedVisionUpdate> visionEstimates) {}
}
