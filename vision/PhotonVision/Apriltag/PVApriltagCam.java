package frc.robot.ShamLib.vision.PhotonVision.Apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.UnaryOperator;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class PVApriltagCam {
  private final PVApriltagIO io;
  private final PVApriltagIO.PVApriltagInputs inputs = new PVApriltagIO.PVApriltagInputs();
  private final String name;
  private final AprilTagFieldLayout fieldLayout;
  private final double trustCutOff;
  private final PhotonPoseEstimator photonPoseEstimator;

  private UnaryOperator<PhotonPipelineResult> preprocess = null;
  private Function<EstimatedRobotPose, TimestampedPoseEstimator.TimestampedVisionUpdate>
      postprocess = this::defaultPostProcess;

  public PVApriltagCam(
      String name,
      ShamLibConstants.BuildMode buildMode,
      Transform3d botToCam,
      AprilTagFieldLayout fieldLayout,
      double trustCutOff) {
    io = getNewIO(buildMode, name);

    photonPoseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, botToCam);

    this.name = name;
    this.fieldLayout = fieldLayout;
    this.trustCutOff = trustCutOff;
  }

  public TimestampedPoseEstimator.TimestampedVisionUpdate defaultPostProcess(
      EstimatedRobotPose estimate) {
    return new TimestampedPoseEstimator.TimestampedVisionUpdate(
        estimate.timestampSeconds, estimate.estimatedPose.toPose2d(), getXYThetaStdDev(estimate));
  }

  public void setPreProcess(UnaryOperator<PhotonPipelineResult> preprocess) {
    this.preprocess = preprocess;
  }

  public void setPostProcess(
      Function<EstimatedRobotPose, TimestampedPoseEstimator.TimestampedVisionUpdate> postprocess) {
    this.postprocess = postprocess;
  }

  public void setPoseEstimationStrategy(PhotonPoseEstimator.PoseStrategy strategy) {
    photonPoseEstimator.setPrimaryStrategy(strategy);
  }

  public void setMultiTagFallbackEstimationStrategy(PhotonPoseEstimator.PoseStrategy strategy) {
    photonPoseEstimator.setMultiTagFallbackStrategy(strategy);
  }

  public void setReferencePose(Pose2d pose) {
    photonPoseEstimator.setReferencePose(pose);
  }

  public void setLastPose(Pose2d pose) {
    photonPoseEstimator.setLastPose(pose);
  }

  public void update() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    var estimate = getLatestEstimate();

    estimate.ifPresent((e) -> Logger.recordOutput(name + "/latestEstimate", e.pose()));
  }

  public Matrix<N3, N1> getXYThetaStdDev(EstimatedRobotPose pose) {
    double totalDistance = 0.0;
    int nonErrorTags = 0;

    // make the std dev greater based on how far away the tags are (trust estimates from further
    // tags less)
    // algorithm from frc6328

    for (var tag : pose.targetsUsed) {
      var tagOnField = fieldLayout.getTagPose(tag.getFiducialId());

      if (tagOnField.isPresent()) {
        totalDistance +=
            pose.estimatedPose
                .toPose2d()
                .getTranslation()
                .getDistance(tagOnField.get().toPose2d().getTranslation());
        nonErrorTags++;
      }
    }
    double avgDistance = totalDistance / nonErrorTags;

    double xyStdDev = Math.pow(avgDistance, 2.0) / nonErrorTags;
    double thetaStdDev = Math.pow(avgDistance, 2.0) / nonErrorTags;

    if (avgDistance >= trustCutOff) {
      return VecBuilder.fill(10000, 10000, 10000);
    }

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  public boolean isConnected() {
    return inputs.isConnected;
  }

  public Optional<TimestampedPoseEstimator.TimestampedVisionUpdate> getLatestEstimate() {
    var raw = inputs.frame;

    if (preprocess != null) {
      raw = preprocess.apply(raw);
    }

    var estimate = photonPoseEstimator.update(raw, inputs.cameraMatrix, inputs.distanceCoeffs);

    return estimate.map(postprocess);
  }

  private PVApriltagIO getNewIO(ShamLibConstants.BuildMode buildMode, String name) {
    return switch (buildMode) {
      case REPLAY -> new PVApriltagIO() {};
      default -> new PVApriltagIOReal(name);
    };
  }
}
