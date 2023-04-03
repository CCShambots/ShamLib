package frc.robot.ShamLib.Candle;

import java.util.ArrayList;
import java.util.List;

public class MultipleColorSegments {
    final List<ColorSegmentInfo> colorSegmentInfoList = new ArrayList<>();

    public MultipleColorSegments(int baseOffset, RGBSegmentInfo... segments) {
        int offset = baseOffset;
        for(RGBSegmentInfo seg : segments) {
            colorSegmentInfoList.add(new ColorSegmentInfo(seg.rgb, seg.numLEDs, offset));
            offset+= seg.numLEDs;
        }
    }

    public MultipleColorSegments(RGBSegmentInfo... segments) {
        this(0, segments);
    }

    static class ColorSegmentInfo {
        final RGB rgb;
        final int numLEDs;
        final int startLED;

        public ColorSegmentInfo(RGB rgb, int numLEDs, int startLED) {
            this.rgb = rgb;
            this.numLEDs = numLEDs;
            this.startLED = startLED;
        }
    }
}
