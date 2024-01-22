package frc.robot.ShamLib.vision;

import frc.robot.ShamLib.ShamLibConstants;
import org.littletonrobotics.junction.Logger;

public class Limelight {
  private final LimelightIO io;
  private final LimelightInputsAutoLogged inputs = new LimelightInputsAutoLogged();
  private final String name;

  public Limelight(String name, ShamLibConstants.BuildMode buildMode) {
    io = getNewIO(buildMode, name);
    this.name = name;
  }

  public Limelight(ShamLibConstants.BuildMode buildMode) {
    this("limelight", buildMode);
  }

  public LimelightIO.LimelightInputs getInputs() {
    return inputs;
  }

  public void update() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public void setLEDState(int state) {
    io.setLEDState(state);
  }

  public void setCamMode(int mode) {
    io.setCamMode(mode);
  }

  public void setPipeline(int pipeline) {
    io.setPipeline(pipeline);
  }

  public void setStream(int mode) {
    io.setStream(mode);
  }

  public void setCrop(double[] crop) {
    io.setCrop(crop);
  }

  public void setLLRobot(double[] values) {
    io.setLLRobot(values);
  }

  private LimelightIO getNewIO(ShamLibConstants.BuildMode buildMode, String name) {
    if (buildMode == ShamLibConstants.BuildMode.REAL) {
      return new LimelightIOReal(name);
    }
    return new LimelightIO() {};
  }
}
