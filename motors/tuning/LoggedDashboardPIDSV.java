package frc.robot.ShamLib.motors.tuning;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class LoggedDashboardPIDSV implements LoggedDashboardInput {
  private final String key;
  private PIDSVGains defaultValue;
  private PIDSVGains value;

  private final LoggableInputs inputs =
      new LoggableInputs() {
        @Override
        public void toLog(LogTable table) {
          table.put(key, value.toArray());
        }

        @Override
        public void fromLog(LogTable table) {
          value.set(table.get(key, defaultValue.toArray()));
        }
      };

  public LoggedDashboardPIDSV(String key, PIDSVGains defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.value = defaultValue.clone();

    SmartDashboard.putNumberArray(key, SmartDashboard.getNumberArray(key, defaultValue.toArray()));

    periodic();

    Logger.registerDashboardInput(this);
  }

  public void setDefault(PIDSVGains defaultValue) {
    this.defaultValue = defaultValue;
  }

  public PIDSVGains getDefault() {
    return defaultValue;
  }

  public void set(PIDSVGains value) {
    SmartDashboard.putNumberArray(key, value.toArray());
  }

  public PIDSVGains get() {
    return value;
  }

  @Override
  public void periodic() {
    if (!Logger.hasReplaySource()) {
      value.set(SmartDashboard.getNumberArray(key, defaultValue.toArray()));
    }
    Logger.processInputs(prefix, inputs);
  }
}
