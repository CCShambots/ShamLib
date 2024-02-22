package frc.robot.ShamLib.Candle;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Consumer;

public class TimedColorFlowCommand extends Command {

  private final Timer timer = new Timer();
  private final int numLEDS;
  private final int startOffset;
  private final Consumer<MultipleColorSegments> setLEDs;
  private final double totalSeconds;
  private final RGB color;
  private final RGB backgroundColor;

  public TimedColorFlowCommand(
      int numLEDS,
      int startOffset,
      Consumer<MultipleColorSegments> setLEDs,
      double totalSeconds,
      RGB color,
      RGB backgroundColor) {
    this.numLEDS = numLEDS;
    this.startOffset = startOffset;
    this.setLEDs = setLEDs;
    this.totalSeconds = totalSeconds;
    this.color = color;
    this.backgroundColor = backgroundColor;
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    double amountElapsed = timer.get() / totalSeconds;

    int numLightsIlluminated = (int) (amountElapsed * numLEDS);

    MultipleColorSegments segs =
        new MultipleColorSegments(
            new RGBSegmentInfo(new RGB(0, 0, 0), startOffset),
            new RGBSegmentInfo(color, numLightsIlluminated),
            new RGBSegmentInfo(backgroundColor, numLEDS - numLightsIlluminated));

    setLEDs.accept(segs);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(totalSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }
}
