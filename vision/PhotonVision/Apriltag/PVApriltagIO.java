package frc.robot.ShamLib.vision.PhotonVision.Apriltag;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonPoseEstimator;

public interface PVApriltagIO {
  @AutoLog
  public class PVApriltagInputs {
    public Pose2d poseEstimate = new Pose2d();
    public double timestamp = 0.0;
    public int[] targetsUsed = new int[0];

    public boolean isConnected = false;
  }

  public default void setEstimationStrategy(PhotonPoseEstimator.PoseStrategy strategy) {}

  public default void setLastPose(Pose2d pose) {}

  public default void setReferencePose(Pose2d pose) {}

  public default void setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy strategy) {}

  public default void updateInputs(PVApriltagInputs inputs) {}
}
