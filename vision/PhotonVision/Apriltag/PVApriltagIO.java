package frc.robot.ShamLib.vision.PhotonVision.Apriltag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface PVApriltagIO {
  @AutoLog
  public class PVApriltagInputs {
    public Pose2d poseEstimate;
    public double timestamp;
    public int[] targetsUsed;
  }

  public default void updateInputs(PVApriltagInputs inputs) {}
}
