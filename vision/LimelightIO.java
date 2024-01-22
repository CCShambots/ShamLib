package frc.robot.ShamLib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {
  @AutoLog
  public class LimelightInputs {
    public String name = "UNKNOWN";

    public int tv = 0;
    public double tx = 0;
    public double ty = 0;
    public double ta = 0;
    public double tl = 0;
    public double cl = 0;
    public double tShort = 0;
    public double tLong = 0;
    public double tHor = 0;
    public double tVert = 0;
    public int getPipe = 0;

    public Pose3d botPose = new Pose3d();

    public int ledMode = 0;
    public int camMode = 0;
    public int stream = 0;
    public double[] crop = new double[4];

    public double[] llPython = new double[0];
  }

  public default void updateInputs(LimelightInputs inputs) {}

  public default void setLEDState(int state) {}

  public default void setCamMode(int mode) {}

  public default void setPipeline(int pipeline) {}

  public default void setStream(int mode) {}

  public default void setCrop(double[] crop) {}

  public default void setLLRobot(double[] values) {}
}
