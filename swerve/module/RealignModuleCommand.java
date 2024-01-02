package frc.robot.ShamLib.swerve.module;

import static frc.robot.ShamLib.ShamLibConstants.Swerve.ALLOWED_STOPPED_MODULE_DIFF;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;

public class RealignModuleCommand extends Command {

  final List<Double> positionList = new ArrayList<>();
  double minAbsoluteValue = 0;

  final Timer timer = new Timer();
  boolean finished = false;

  final SwerveModule module;

  public RealignModuleCommand(SwerveModule module) {
    this.module = module;
  }

  @Override
  public void initialize() {
    System.out.println("Irregularity detected on module " + module.getModuleName());
    positionList.clear();
    minAbsoluteValue = module.getAbsoluteAngle().getDegrees();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double thisValue = module.getAbsoluteAngle().getDegrees();

    positionList.add(thisValue);

    if (Math.abs(thisValue - minAbsoluteValue) > ALLOWED_STOPPED_MODULE_DIFF
        || Math.abs(module.getDriveMotorRate()) > 0.1
        || Math.abs(module.getTurnMotorVelo()) > 0.1) {
      System.out.println("FAILED TO CORRECT MODULE IRREGULARITY");
      cancel();
    }

    if (timer.get() > 1) finished = true;

    if (thisValue < minAbsoluteValue) minAbsoluteValue = thisValue;
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      double average = positionList.stream().mapToDouble((e) -> e).average().orElse(0);

      System.out.println(
          "CORRECTED "
              + module.getModuleName()
              + " TO: "
              + average
              + " from "
              + module.getDriveMotorPosition());

      module.resetAngle(Rotation2d.fromDegrees(average));
    }

    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
