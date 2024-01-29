package frc.robot.ShamLib.vision.PhotonVision.Apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface PVApriltagIO {
  @AutoLog
  public class PVApriltagInputs {
    public Pose3d[] cameraPoseEstimates = new Pose3d[0];
    public int[] cameraPoseEstimateIDs = new int[0];
    public Pose3d bestCameraPoseEstimate = new Pose3d();
    public int bestTagID = 0;

    public Pose3d multiTagPoseEstimate = new Pose3d();

    public boolean hasTarget = false;

    public double timestamp = 0.0;
  }

  public default void updateInputs(PVApriltagInputs inputs) {}
}
