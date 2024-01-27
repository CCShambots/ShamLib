package frc.robot.ShamLib.vision.Northstar;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.ShamLib.ShamLibConstants;
import org.littletonrobotics.junction.Logger;

public class NorthstarCamera {
    private final NorthstarIO io;
    private final NorthstarIO.NorthstarInputs inputs = new NorthstarIO.NorthstarInputs();
    private final String identifier;

    public NorthstarCamera(String identifier, double apriltagWidth, AprilTagFieldLayout fieldLayout, ShamLibConstants.BuildMode buildMode) {
        this(identifier, apriltagWidth, fieldLayout, NorthstarIO.CameraSettings.DEFAULT, buildMode);
    }

    public NorthstarCamera(String identifier, double apriltagWidth, AprilTagFieldLayout fieldLayout, NorthstarIO.CameraSettings settings, ShamLibConstants.BuildMode buildMode) {
        io = switch (buildMode) {
            case REPLAY -> new NorthstarIO() {};
            default -> new NorthstarIOReal(identifier, settings, apriltagWidth, fieldLayout);
        };

        this.identifier = "NorthStarInstances/ " + identifier;

        io.updateInputs(inputs);
    }
    public void update() {
        io.updateInputs(inputs);
        Logger.processInputs(identifier, inputs);
    }
}
