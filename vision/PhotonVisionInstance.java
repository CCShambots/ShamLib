package frc.robot.ShamLib.vision;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Filesystem;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonVisionInstance {
    private Transform3d cameraToRobotPose;
    private PhotonCamera camera;
    private RobotPoseEstimator poseEstimator;

    /**
     * Constructor for PhotonVision pose estimation
     * @param cameraPose The pose of the camera relative to the robot
     */
    public PhotonVisionInstance(Pose3d cameraPose, String camName) throws IOException {
        this.cameraToRobotPose = new Transform3d(new Pose3d(), cameraPose);
        this.camera = new PhotonCamera(camName);

        this.poseEstimator = new RobotPoseEstimator(
                new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("2023-chargedup.json")),
                RobotPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, //TODO: Which of these is actually best?
                List.of(new Pair<>(camera, cameraToRobotPose))
        );
    }

    public void update(Pose3d referencePose) {
        poseEstimator.setReferencePose(referencePose);
    }
    
    public Pose3d getPose3d() {
        return poseEstimator.update().get().getFirst();
    }

    public Pose3d getPose3d(Pose3d referencePose) {
        update(referencePose);
        return getPose3d();
    }

}
