package frc.robot.ShamLib.swerve.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.ShamLib.swerve.module.SwerveModule;
import java.util.List;

public class SwerveOdometrySim implements SwerveOdometry {
  private Pose2d pose = new Pose2d();
  private SwerveDriveKinematics kDriveKinematics;
  private List<SwerveModule> modules;

  public SwerveOdometrySim(SwerveDriveKinematics kDriveKinematics, List<SwerveModule> modules) {
    this.kDriveKinematics = kDriveKinematics;
    this.modules = modules;
  }

  @Override
  public Pose2d getPose() {
    return pose;
  }

  @Override
  public void resetPose(Pose2d newPose, SwerveModulePosition[] modulePositions) {
    pose = newPose;
  }

  @Override
  public void updatePose() {
    // Update odometry manually
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] = modules.get(i).getPositionDelta();
    }
    // The twist represents the motion of the robot since the last
    // loop cycle in x, y, and theta based only on the modules,
    // without the gyro. The gyro is always disconnected in simulation.
    var twist = kDriveKinematics.toTwist2d(wheelDeltas);

    // Apply the twist (change since last loop cycle) to the current pose
    pose = pose.exp(twist);
  }
}
