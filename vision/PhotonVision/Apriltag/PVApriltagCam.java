package frc.robot.ShamLib.vision.PhotonVision.Apriltag;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.vision.Limelight.LimelightIO;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class PVApriltagCam {
  private final PVApriltagIO io;
  private final PVApriltagInputsAutoLogged inputs = new PVApriltagInputsAutoLogged();
  private final String name;

  public PVApriltagCam(String name, ShamLibConstants.BuildMode buildMode) {
      io = getNewIO(buildMode, name);
      this.name = name;
  }

  public void update() {
      io.updateInputs(inputs);
      Logger.processInputs(name, inputs);
  }

  public TimestampedPose[] getAllEstimates() {
      double timestamp = inputs.timestamp;
      return Arrays.stream(inputs.cameraPoseEstimates)
              .map((pose) -> new TimestampedPose(pose, timestamp))
              .toArray(TimestampedPose[]::new);
  }

  public TimestampedPose getMultiTagEstimate() {
      return new TimestampedPose(inputs.multiTagPoseEstimate, inputs.timestamp);
  }

  public TimestampedPose getBestPoseEstimate() {
      return new TimestampedPose(inputs.bestCameraPoseEstimate, inputs.timestamp);
  }

  private PVApriltagIO getNewIO(ShamLibConstants.BuildMode buildMode, String name) {
      return switch (buildMode) {
          case REPLAY -> new PVApriltagIO() {};
          default -> new PVApriltagIOReal(name);
      };
  }

  public record TimestampedPose(
          Pose3d pose,
          double timestamp
  ) {
      //FRC6328 std dev algo, multiply by coeffs after
      public static Pair<Double, Double> getXYThetaStdDev(TimestampedPose[] poses, Pose3d from) {
          double totalDistance = 0.0;
          for (TimestampedPose tagPose : poses) {
              totalDistance += tagPose.pose().getTranslation().getDistance(from.getTranslation());
          }
          double avgDistance = totalDistance / poses.length;

          double xyStdDev = Math.pow(avgDistance, 2.0) / poses.length;
          double thetaStdDev = Math.pow(avgDistance, 2.0) / poses.length;

          return new Pair<>(xyStdDev, thetaStdDev);
      }
  }
}
