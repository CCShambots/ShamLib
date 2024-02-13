package frc.robot.ShamLib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.Optional;

public class AllianceManager {
  public static DriverStation.Alliance alliance = DriverStation.Alliance.Red;
  public static boolean overrideAlliance = false;

  public static void applyAlliance(Optional<DriverStation.Alliance> newAlliance) {
    if (!overrideAlliance && newAlliance.isPresent()) {
      alliance = newAlliance.get();
    }
  }

  public static InstantCommand switchAlliance() {
    return new WhileDisabledInstantCommand(
        () -> {
          alliance = alliance == Alliance.Red ? Alliance.Blue : Alliance.Red;
          overrideAlliance = true;
        });
  }

  public static InstantCommand syncAlliance() {
    return new WhileDisabledInstantCommand(
        () -> {
          applyAlliance(DriverStation.getAlliance());
          overrideAlliance = false;
        });
  }
}
