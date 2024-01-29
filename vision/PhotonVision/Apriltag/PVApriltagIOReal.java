package frc.robot.ShamLib.vision.PhotonVision.Apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.ShamLib.util.GeomUtil;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PVApriltagIOReal implements PVApriltagIO {
  private final PhotonCamera camera;

  public PVApriltagIOReal(String camName) {
    camera = new PhotonCamera(camName);
  }

  @Override
  public void updateInputs(PVApriltagInputs inputs) {
    var latest = camera.getLatestResult();

    Pose3d[] poses = new Pose3d[latest.targets.size()];
    int[] ids = new int[latest.targets.size()];

    for (int i = 0; i < latest.targets.size(); i++) {
      PhotonTrackedTarget target = latest.targets.get(i);
      poses[i] = GeomUtil.transform3dToPose3d(target.getBestCameraToTarget());
      ids[i] = target.getFiducialId();
    }

    inputs.timestamp = latest.getTimestampSeconds();

    inputs.cameraPoseEstimates = poses;
    inputs.cameraPoseEstimateIDs = ids;

    if (latest.hasTargets()) {
      inputs.bestCameraPoseEstimate =
          GeomUtil.transform3dToPose3d(latest.getBestTarget().getBestCameraToTarget());
      inputs.bestTagID = latest.getBestTarget().getFiducialId();

      inputs.multiTagPoseEstimate =
          GeomUtil.transform3dToPose3d(latest.getMultiTagResult().estimatedPose.best);

      inputs.hasTarget = true;
    } else {
      inputs.hasTarget = false;
      inputs.bestTagID = -1;
    }

    inputs.isConnected = camera.isConnected();
  }
}
