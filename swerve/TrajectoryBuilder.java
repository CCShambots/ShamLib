package frc.robot.ShamLib.swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryBuilder {

    private final List<PathPoint> points = new ArrayList<>();
    private Rotation2d lastHolonomicRotation;
    private final PathConstraints constraints;

    public TrajectoryBuilder(Pose2d initialPose, ChassisSpeeds initialSpeeds, PathConstraints constraints) {
        this.constraints = constraints;

        points.add(new PathPoint(
                initialPose.getTranslation(),
                getDirectionOfMovement(initialSpeeds),
                initialPose.getRotation(),
                getChassisSpeed(initialSpeeds)
        ));

        lastHolonomicRotation = initialPose.getRotation();

    }

    public PathPlannerTrajectory build() {

        if(points.size() >2) {
            PathPoint[] pointsArr = new PathPoint[points.size()-2];

            for(int i = 2; i<pointsArr.length; i++) {
                pointsArr[i] = points.get(i);
            }

            return PathPlanner.generatePath(constraints, points.get(0), points.get(1), pointsArr);

        } else return PathPlanner.generatePath(constraints, points.get(0), points.get(1));
    }

    /**
     * Run a spline that will keep the bot at the same heading
     * @param translation translation target
     * @param endTangent the end tangent of the spline
     * @return this
     */
    public TrajectoryBuilder splineToConstantHeading(Translation2d translation, Rotation2d endTangent) {
        points.add(new PathPoint(
                        translation,
                        endTangent,
                        lastHolonomicRotation
                )
        );

        return this;
    }

    /**
     * Run a spline to a specific heading
     * @param pose the pose target (including holonomic rotation)
     * @param endTangent the end tangent of the spline
     * @return this
     */
    public TrajectoryBuilder splineToHeading(Pose2d pose, Rotation2d endTangent) {
        points.add(new PathPoint(
                        pose.getTranslation(),
                        endTangent,
                        pose.getRotation()
                )
        );

        lastHolonomicRotation = pose.getRotation();

        return this;
    }

    public double getChassisSpeed(ChassisSpeeds speeds) {
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    public Rotation2d getDirectionOfMovement(ChassisSpeeds speeds) {
        return new Rotation2d(Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond));
    }

}
