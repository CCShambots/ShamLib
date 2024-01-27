package frc.robot.ShamLib.vision.PhotonVision.Apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.ShamLib.util.GeomUtil;
import org.photonvision.PhotonCamera;

public class PVApriltagIOReal implements PVApriltagIO {
  private final PhotonCamera camera;

  public PVApriltagIOReal(String camName) {
    camera = new PhotonCamera(camName);
  }

  @Override
  public void updateInputs(PVApriltagInputs inputs) {
    var latest = camera.getLatestResult();

    inputs.timestamp = latest.getTimestampSeconds();

    inputs.cameraPoseEstimates =
        latest.targets.stream()
            .map((e) -> GeomUtil.transform3dToPose3d(e.getBestCameraToTarget()))
            .toArray(Pose3d[]::new);

    if (latest.hasTargets()) {
      inputs.bestCameraPoseEstimate =
          GeomUtil.transform3dToPose3d(latest.getBestTarget().getBestCameraToTarget());

      inputs.multiTagPoseEstimate =
          GeomUtil.transform3dToPose3d(latest.getMultiTagResult().estimatedPose.best);

      inputs.hasTarget = true;
    } else {
      inputs.hasTarget = false;
    }
  }
}
