package frc.robot.ShamLib.vision.Northstar;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class NorthstarIOReal implements NorthstarIO {
    private final DoubleArraySubscriber observationSubscriber;

    public NorthstarIOReal(String identifier, CameraSettings settings, double aprilTagWidth, AprilTagFieldLayout fieldLayout) {
        System.out.println("[Init] Creating AprilTagVisionIONorthstar (" + identifier + ")");
        var northstarTable = NetworkTableInstance.getDefault().getTable(identifier);

        var configTable = northstarTable.getSubTable("config");
        configTable.getIntegerTopic("camera_id").publish().set(settings.cameraId());
        configTable.getIntegerTopic("camera_resolution_width").publish().set(settings.cameraResolutionWidth());
        configTable.getIntegerTopic("camera_resolution_height").publish().set(settings.cameraResolutionHeight());
        configTable.getIntegerTopic("camera_auto_exposure").publish().set(settings.cameraAutoExposure());
        configTable.getIntegerTopic("camera_exposure").publish().set(settings.cameraExposure());
        configTable.getIntegerTopic("camera_gain").publish().set(settings.cameraGain());
        configTable.getDoubleTopic("fiducial_size_m").publish().set(aprilTagWidth);
        try {
            configTable
                    .getStringTopic("tag_layout")
                    .publish()
                    .set(new ObjectMapper().writeValueAsString(fieldLayout));
        } catch (JsonProcessingException e) {
            throw new RuntimeException("Failed to serialize AprilTag layout JSON for Northstar");
        }

        var outputTable = northstarTable.getSubTable("output");
        observationSubscriber =
                outputTable
                        .getDoubleArrayTopic("observations")
                        .subscribe(
                                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    }

    @Override
    public void updateInputs(NorthstarInputs inputs) {
        var queue = observationSubscriber.readQueue();
        inputs.timestamps = new double[queue.length];
        inputs.frames = new double[queue.length][];
        for (int i = 0; i < queue.length; i++) {
            inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
            inputs.frames[i] = queue[i].value;
        }
    }
}
