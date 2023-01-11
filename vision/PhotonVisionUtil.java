package frc.robot.ShamLib.vision;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonVisionUtil {
    private Transform3d cameraToRobotPose;
    private static List<Pose3d> tags = List.of(

    );

    /**
     * Constructor for PhotonVision pose estimation
     * @param cameraPose The pose of the camera relative to the robot
     */
    public PhotonVisionUtil(Pose3d cameraPose) {
        this.cameraToRobotPose = new Transform3d(cameraPose, new Pose3d());
    }
    
    public Pose2d getRobotPoseFromTarget(PhotonTrackedTarget target) {            
        
        Transform3d camToTarget = target.getBestCameraToTarget(); //Get the transform from the camera to the target
        RobotPoseEstimator poseEstimator = new RobotPoseEstimator(
                new AprilTagFieldLayout(),
                RobotPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, //TODO: Which of these is actually best?

        )

        You'll put that into the deploy folder in your project and then do AprilTagFieldLayout(GetDeployDirectory() + "/file_name.json") (translate the syntax to your respective language)
        
        Pose3d camPose3d = tags.get(target.getFiducialId()).transformBy(camToTarget.inverse()); //Derive the camera pose from the target's pose
        
        Pose3d robotPose3d = camPose3d.transformBy(cameraToRobotPose); //Get the robot's pose from the camera's location
            
        return robotPose3d.toPose2d();

    }

}
