package frc.robot.ShamLib.motors.tuning;

import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import java.util.function.BooleanSupplier;

public class LoggedTunablePIDSV {

  private final LoggedDashboardPIDSV dashboardPIDSV;
  private final BooleanSupplier allowTuningSupplier;

  public LoggedTunablePIDSV(
      String name, PIDSVGains defaultValue, BooleanSupplier allowTuningSupplier) {
    this.allowTuningSupplier = allowTuningSupplier;

    dashboardPIDSV = new LoggedDashboardPIDSV(name, defaultValue);
  }

  public PIDSVGains get() {
    if (allowTuningSupplier.getAsBoolean()) {
      return dashboardPIDSV.get();
    } else {
      // reset dashboard number to default and return the default
      PIDSVGains defaultValue = dashboardPIDSV.getDefault();
      dashboardPIDSV.set(defaultValue);
      return defaultValue;
    }
  }

  public void set(PIDSVGains value) {
    dashboardPIDSV.set(value);
  }

  public void setDefault(PIDSVGains value) {
    dashboardPIDSV.setDefault(value);
  }

  public PIDSVGains getDefault() {
    return dashboardPIDSV.getDefault();
  }

  public boolean hasChanged(PIDSVGains val) {
    return !get().equals(val);
  }
}
