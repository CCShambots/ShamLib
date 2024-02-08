package frc.robot.ShamLib.vision.PhotonVision.Apriltag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonPoseEstimator;

public interface PVApriltagIO {
  @AutoLog
  public class PVApriltagInputs {
    public Pose2d poseEstimate;
    public double timestamp;
    public int[] targetsUsed;

    public boolean isConnected;
  }

  public default void setEstimationStrategy(PhotonPoseEstimator.PoseStrategy strategy) {}

  public default void setLastPose(Pose2d pose) {}

  public default void setReferencePose(Pose2d pose) {}
  public default void setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy strategy) {}
  public default void updateInputs(PVApriltagInputs inputs) {}
}
