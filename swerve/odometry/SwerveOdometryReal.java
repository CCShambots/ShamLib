package frc.robot.ShamLib.swerve.odometry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;

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
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  @Override
  public void resetPose(Pose2d newPose, SwerveModulePosition[] modulePositions) {
    odometry.resetPosition(newPose.getRotation(), modulePositions, newPose);
  }

  @Override
  public void updatePose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {}
}
