package frc.robot.ShamLib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightIOReal implements LimelightIO {

  private final double[] poseDefault = new double[6];
  private final double[] cropDefault = new double[4];
  private final NetworkTable table;
  private final String name;

  public LimelightIOReal(String name) {
    this.name = name;
    table = getLimeLightTable(name);
  }

  public LimelightIOReal() {
    this("limelight");
  }

  @Override
  public void updateInputs(LimelightInputs inputs) {
    inputs.name = name;

    inputs.tv = getInt("tv", 0);
    inputs.tx = getDouble("tx", 0);
    inputs.ty = getDouble("ty", 0);
    inputs.ta = getDouble("ta", 0);
    inputs.tl = getDouble("tl", 0);
    inputs.cl = getDouble("cl", 0);
    inputs.tShort = getDouble("tShort", 0);
    inputs.tLong = getDouble("tLong", 0);
    inputs.tHor = getDouble("tHor", 0);
    inputs.tVert = getDouble("tVert", 0);
    inputs.getPipe = getInt("getpipe", 0);

    double[] pose = getDoubleArray("botpose", poseDefault);

    inputs.botPose =
        new Pose3d(
            new Translation3d(pose[0], pose[1], pose[2]),
            new Rotation3d(pose[3], pose[4], pose[5]));

    inputs.ledMode = getInt("ledMode", 0);
    inputs.camMode = getInt("camMode", 0);
    inputs.stream = getInt("stream", 0);

    inputs.crop = getDoubleArray("crop", cropDefault);

    inputs.llPython = getDoubleArray("llpython", new double[0]);
  }

  @Override
  public void setLEDState(int state) {
    setInt("ledMode", state);
  }

  @Override
  public void setCamMode(int mode) {
    setInt("camMode", mode);
  }

  @Override
  public void setPipeline(int pipeline) {
    setInt("pipeline", pipeline);
  }

  @Override
  public void setStream(int mode) {
    setInt("stream", mode);
  }

  @Override
  public void setCrop(double[] crop) {
    setDoubleArray("crop", crop);
  }

  @Override
  public void setLLRobot(double[] values) {
    setDoubleArray("llrobot", values);
  }

  private double getDouble(String key, double def) {
    return table.getEntry(key).getDouble(def);
  }

  private int getInt(String key, int def) {
    return (int) table.getEntry(key).getInteger(def);
  }

  private double[] getDoubleArray(String key, double[] def) {
    return table.getEntry(key).getDoubleArray(def);
  }

  private void setInt(String key, int value) {
    table.getEntry(key).setNumber(value);
  }

  private void setDoubleArray(String key, double[] value) {
    table.getEntry(key).setDoubleArray(value);
  }

  private NetworkTable getLimeLightTable(String tableID) {
    return NetworkTableInstance.getDefault().getTable(tableID);
  }
}
