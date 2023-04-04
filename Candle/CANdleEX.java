package frc.robot.ShamLib.Candle;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import frc.robot.Constants;

public class CANdleEX extends CANdle {

    private final int ledCount;

    /**
     * Constructs a new CANdleEX object
     * @param canID id of the candle
     * @param brightness brightness [0.0-1.0]
     * @param ledCount number of leds in the string
     */
    public CANdleEX(int canID, double brightness, int ledCount) {
        super(canID);

        this.ledCount = ledCount;

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = brightness;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        configAllSettings(configAll, 100);

    }

    public int getLedCount() {
        return ledCount;
    }

    public void setBrightness(double percent) { configBrightnessScalar(percent, 0); }

    public void setLEDs(RGB values) {
        clearAnimation(0);
        setLEDs(values.R, values.G, values.B);
    }

    public void setLEDs(MultipleColorSegments segs) {
        clearAnimation(0);
        for(MultipleColorSegments.ColorSegmentInfo info : segs.colorSegmentInfoList) {
            RGB rgb = info.rgb;
            setLEDs(rgb.R, rgb.G, rgb.B, 0, info.startLED, info.numLEDs);
        }
    }


}
