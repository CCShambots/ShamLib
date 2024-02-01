package frc.robot.ShamLib.vision.PhotonVision.Apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;
import frc.robot.ShamLib.util.GeomUtil;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class PVApriltagCam {
  private final PVApriltagIO io;
  private final PVApriltagInputsAutoLogged inputs = new PVApriltagInputsAutoLogged();
  private final String name;
  private final AprilTagFieldLayout fieldLayout;

  public PVApriltagCam(
      String name, ShamLibConstants.BuildMode buildMode, AprilTagFieldLayout fieldLayout) {
    io = getNewIO(buildMode, name);
    this.name = name;
    this.fieldLayout = fieldLayout;
  }

  public void update() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  private Matrix<N3, N1> getXYThetaStdDev(Pose3d pose, int[] tagIds) {
    double totalDistance = 0.0;
    int nonErrorTags = 0;

    // make the std dev greater based on how far away the tags are (trust estimates from further
    // tags less)
    // algorithm from frc6328

    for (int tagId : tagIds) {
      var tagOnField = fieldLayout.getTagPose(tagId);

      if (tagOnField.isPresent()) {
        totalDistance += pose.getTranslation().getDistance(tagOnField.get().getTranslation());
        nonErrorTags++;
      }
    }
    double avgDistance = totalDistance / nonErrorTags;

    double xyStdDev = Math.pow(avgDistance, 2.0) / nonErrorTags;
    double thetaStdDev = Math.pow(avgDistance, 2.0) / nonErrorTags;

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  public boolean isConnected() {
    return inputs.isConnected;
  }

  public ArrayList<TimestampedPoseEstimator.TimestampedVisionUpdate> getAllEstimates() {
    double timestamp = inputs.timestamp;
    Pose3d[] estimates = inputs.cameraPoseEstimates;
    int[] ids = inputs.cameraPoseEstimateIDs;
    ArrayList<TimestampedPoseEstimator.TimestampedVisionUpdate> out = new ArrayList<>();

    if (!inputs.hasTarget) return out;

    for (int i = 0; i < estimates.length; i++) {
      Transform3d estimate = GeomUtil.pose3dToTransform3d(estimates[i]);
      var tagPose = fieldLayout.getTagPose(ids[i]);

      if (tagPose.isPresent()) {
        Pose3d pose = tagPose.get().transformBy(estimate.inverse());

        // exit if any estimates are >10cm outside the field
        if (pose.getX() > fieldLayout.getFieldLength() + 0.1
            || pose.getX() < -0.1
            || pose.getY() > fieldLayout.getFieldWidth() + 0.1
            || pose.getY() < -0.1) {
          continue;
        }

        out.add(
            new TimestampedPoseEstimator.TimestampedVisionUpdate(
                timestamp, pose.toPose2d(), getXYThetaStdDev(pose, new int[] {ids[i]})));
      }
    }

    return out;
  }

  public Optional<TimestampedPoseEstimator.TimestampedVisionUpdate> getMultiTagEstimate() {
    if (inputs.hasTarget) {
      return Optional.of(
          new TimestampedPoseEstimator.TimestampedVisionUpdate(
              inputs.timestamp,
              inputs.multiTagPoseEstimate.toPose2d(),
              getXYThetaStdDev(inputs.multiTagPoseEstimate, inputs.cameraPoseEstimateIDs)));
    }

    return Optional.empty();
  }

  public Optional<TimestampedPoseEstimator.TimestampedVisionUpdate> getBestPoseEstimate() {
    if (inputs.hasTarget) {
      return Optional.of(
          new TimestampedPoseEstimator.TimestampedVisionUpdate(
              inputs.timestamp,
              inputs.bestCameraPoseEstimate.toPose2d(),
              getXYThetaStdDev(inputs.bestCameraPoseEstimate, new int[] {inputs.bestTagID})));
    }

    return Optional.empty();
  }

  private PVApriltagIO getNewIO(ShamLibConstants.BuildMode buildMode, String name) {
    return switch (buildMode) {
      case REPLAY -> new PVApriltagIO() {};
      default -> new PVApriltagIOReal(name);
    };
  }
}
