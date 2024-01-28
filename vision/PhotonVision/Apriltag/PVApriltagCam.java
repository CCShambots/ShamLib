package frc.robot.ShamLib.vision.PhotonVision.Apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.util.GeomUtil;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class PVApriltagCam {
  private final PVApriltagIO io;
  private final PVApriltagInputsAutoLogged inputs = new PVApriltagInputsAutoLogged();
  private final String name;
  private final AprilTagFieldLayout fieldLayout;

  public PVApriltagCam(String name, ShamLibConstants.BuildMode buildMode, AprilTagFieldLayout fieldLayout) {
      io = getNewIO(buildMode, name);
      this.name = name;
      this.fieldLayout = fieldLayout;
  }

  public void update() {
      io.updateInputs(inputs);
      Logger.processInputs(name, inputs);
  }

  public ArrayList<TimestampedPose> getAllEstimates() {
      double timestamp = inputs.timestamp;
      Pose3d[] estimates = inputs.cameraPoseEstimates;
      int[] ids = inputs.cameraPoseEstimateIDs;
      ArrayList<TimestampedPose> out = new ArrayList<>();


      for (int i = 0; i < estimates.length; i++) {
          Transform3d estimate = GeomUtil.pose3dToTransform3d(estimates[i]);
          var tagPose = fieldLayout.getTagPose(ids[i]);

          if (tagPose.isPresent()) {
              Pose3d pose = tagPose.get().transformBy(estimate.inverse());

              //exit if any estimates are >10cm outside the field
              if (pose.getX() > fieldLayout.getFieldLength() + 0.1 || pose.getX() < -0.1 || pose.getY() > fieldLayout.getFieldWidth() + 0.1 || pose.getY() < -0.1) {
                  continue;
              }

              out.add(new TimestampedPose(pose, timestamp));
          }

      }

      return out;
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
