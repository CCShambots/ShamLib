package frc.robot.ShamLib.vision.Northstar;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface NorthstarIO {

    public class NorthstarInputs implements LoggableInputs {
        double[] timestamps = new double[0];
        double[][] frames = new double[0][];

        @Override
        public void toLog(LogTable table) {
            table.put("Timestamps", timestamps);
            table.put("FrameCount", frames.length);
            for (int i = 0; i < frames.length; i++) {
                table.put("Frame/" + i, frames[i]);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            timestamps = table.get("Timestamps", new double[] {0.0});
            int frameCount = table.get("FrameCount", 0);
            frames = new double[frameCount][];
            for (int i = 0; i < frameCount; i++) {
                frames[i] = table.get("Frame/" + i, new double[] {});
            }
        }
    }
}
