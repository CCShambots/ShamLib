package frc.robot.ShamLib.vision.PhotonVision.Apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PVApriltagIOReal implements PVApriltagIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  public PVApriltagIOReal(String camName, Transform3d botToCamera, AprilTagFieldLayout layout) {
    camera = new PhotonCamera(camName);
    poseEstimator =
        new PhotonPoseEstimator(
            layout, PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS, camera, botToCamera);
  }

  @Override
  public void setEstimationStrategy(PhotonPoseEstimator.PoseStrategy strategy) {
    poseEstimator.setPrimaryStrategy(strategy);
  }

  @Override
  public void setLastPose(Pose2d pose) {
    poseEstimator.setLastPose(pose);
  }

  @Override
  public void setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy strategy) {
    poseEstimator.setMultiTagFallbackStrategy(strategy);
  }

  @Override
  public void setReferencePose(Pose2d pose) {
    poseEstimator.setReferencePose(pose);
  }

  @Override
  public void updateInputs(PVApriltagInputs inputs) {
    var estimate = poseEstimator.update();
    if (estimate.isPresent()) {
      inputs.poseEstimate = estimate.get().estimatedPose.toPose2d();
      inputs.timestamp = estimate.get().timestampSeconds;
      int[] tags = new int[estimate.get().targetsUsed.size()];

      for (int i = 0; i < tags.length; i++) {
        tags[i] = estimate.get().targetsUsed.get(i).getFiducialId();
      }

      inputs.targetsUsed = tags;
    }

    inputs.isConnected = camera.isConnected();
  }
}
