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

public class SwerveTimestampedOdometrySim implements SwerveOdometry {
  private final TimestampedPoseEstimator estimator;
  private final SwerveDriveKinematics kinematics;
  private final List<SwerveModule> modules;

  public SwerveTimestampedOdometrySim(
      TimestampedPoseEstimator estimator,
      SwerveDriveKinematics kinematics,
      List<SwerveModule> modules) {
    this.estimator = estimator;
    this.kinematics = kinematics;
    this.modules = modules;
  }

  @Override
  public void resetPose(Pose2d newPose, SwerveModulePosition[] modulePositions) {
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

    estimator.addDriveData(Timer.getFPGATimestamp(), twist);
  }
}
