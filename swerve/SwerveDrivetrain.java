package frc.robot.ShamLib.swerve;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.ShamLib.PIDGains;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import static frc.robot.Constants.SwerveDrivetrain.*;

public class SwerveDrivetrain extends SubsystemBase {

    private Map<String, SwerveModule> modules;
    private int numModules = 0;
    private WPI_Pigeon2 gyro;
    private double rotationOffset;
    private Rotation2d holdAngle;

    protected final PIDController thetaHoldControllerTele, thetaHoldControllerAuto, xHoldController, yHoldController;
    private SwerveDrivePoseEstimator odometry;

    private boolean fieldRelative = true;

    protected Field2d field;
    private boolean extraTelemetry;

    /**
     * Constructor for your typical swerve drivetarin with odometry compatible with vision pose estimation
     * @param pigeon2ID CAN idea of the pigeon 2 gyro
     * @param moduleStdDevs Kalman filter standard deviations for the swerve modules
     * @param gyroStdDevs Kalman filter standard deviations for the gyro
     * @param visionStdDevs Kalman filter standard deviations for vision measurements
     * @param teleThetaGains PID gains for the angle hold controller in teleop
     * @param autoThetaGains PID gains for the angle hold controller in autonomous
     * @param translationGains PID gains for the trans
     * @param extraTelemetry whether to send additional telemetry data, like vision pose measurements, trajectory data, and module poses
     * @param moduleInfos Array of module infos, one for each module
     */
    public SwerveDrivetrain(int pigeon2ID, Matrix<N3, N1> moduleStdDevs, Matrix<N1, N1> gyroStdDevs, Matrix<N3, N1> visionStdDevs,
                            PIDGains teleThetaGains, PIDGains autoThetaGains, PIDGains translationGains, boolean extraTelemetry,
                            ModuleInfo... moduleInfos) {

        //TODO: Construct kDriveKinematics in here
        this.extraTelemetry = extraTelemetry;

        //Apply the gains passed in the constructors
        thetaHoldControllerTele =  teleThetaGains.applyToController();

        thetaHoldControllerAuto = autoThetaGains.applyToController();
        xHoldController = translationGains.applyToController();
        yHoldController = translationGains.applyToController();

        gyro = new WPI_Pigeon2(pigeon2ID);

        modules = new HashMap<>();
        for(ModuleInfo i : moduleInfos) {
            numModules++;
            modules.put("Module " + numModules, new SwerveModule("Module-" + numModules, i.turnMotorID, i.driveMotorID, i.encoderMotorID, i.encoderOffset, i.offset));
        }

        gyro.configFactoryDefault();

        rotationOffset = getGyroHeading();
        holdAngle = new Rotation2d(rotationOffset);
        thetaHoldControllerTele.setTolerance(Math.toRadians(1.5));
        
        odometry = new SwerveDrivePoseEstimator(getCurrentAngle(), new Pose2d(), kDriveKinematics, moduleStdDevs, gyroStdDevs, visionStdDevs);

        thetaHoldControllerTele.enableContinuousInput(-Math.PI, Math.PI);
        thetaHoldControllerAuto.enableContinuousInput(-Math.PI, Math.PI);
        field = new Field2d();
    }


//            visionPoseEstimation = ComputerVisionUtil.estimateFieldToRobot(
//                    LIMELIGHT_HEIGHT, GOAL_HEIGHT, LIMELIGHT_ANGLE, Limelight.getInstance().getYOffset().getRadians(), Limelight.getInstance().getXOffset().plus(getRotaryAngle.get()),
//                    getCurrentAngle(), Geometry.getCurrentTargetPose(getDrivetrainAngle.get(), getRotaryAngle.get(), getLimelightXOffsetAngle.get()),
//                    new Transform2d(new Translation2d(), new Rotation2d())
//            );
        

    public void addVisionMeasurement(Pose2d pose) {
        if (extraTelemetry) field.getObject("vision").setPose(pose);

        odometry.addVisionMeasurement(pose, Timer.getFPGATimestamp());
    }


    /**
     * Updates the odometry pose estimator.
     * THIS MUST BE CALLED PERIODICALLY
     */
    public void updateOdometry() {
        odometry.update(getCurrentAngle(), getModuleStates());
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[numModules];

        for(int i = 0; i < modules.size(); i++) {
            states[i] = modules.get(i).getCurrentState();
        }

        return states;
    }

    private void updateField2dObject() {
        Pose2d robotPose = getPose();
        field.setRobotPose(robotPose);

        if(extraTelemetry) {
            //Send each of the module poses to the dashboard as well
            for(Entry<String, SwerveModule> e : modules.entrySet()) {
                field.getObject(e.getKey()).setPose(calculateModulePose(e.getValue(), robotPose));
            }
        }

    }

    private Pose2d calculateModulePose(SwerveModule module, Pose2d robotPose) {
        SwerveModuleState state = module.getCurrentState();
        Translation2d offset = module.getModuleOffset();

        Pose2d pose = new Pose2d(robotPose.getTranslation().plus(offset), robotPose.getRotation().plus(state.angle));

        return pose;
    }

    public void drive(ChassisSpeeds speeds, boolean allowHoldAngleChange) {
        if(speeds.omegaRadiansPerSecond == 0 && !thetaHoldControllerTele.atSetpoint()) {
            speeds.omegaRadiansPerSecond += thetaHoldControllerTele.calculate(getCurrentAngle().getRadians());
            if(Math.abs(Math.toDegrees(speeds.omegaRadiansPerSecond)) < 4) {
                speeds.omegaRadiansPerSecond = 0;
            }

        } else if(allowHoldAngleChange) setHoldAngle(getCurrentAngle());

        SwerveModuleState[] swerveModuleStates = kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_LINEAR_SPEED);

        setModuleStates(swerveModuleStates);
    }

    /**
     * Sets the target state of each swerve module based on the input array
     * @param states array of swerve module states
     */
    public void setModuleStates(SwerveModuleState[] states) {
        for(int i = 0; i<states.length; i++) {
            modules.get("Module " + (i + 1)).setDesiredState(states[i]);
        }
    }

    public void setAllModules(SwerveModuleState state) {
        for(SwerveModule module : modules.values()) {
            module.setDesiredState(state);
        }
    }

    /**
     * Finds the angle of the robot in radians (limited -PI to PI)
     * @return Robot angle
     */
    public Rotation2d getCurrentAngle(){
        double angle = getGyroHeading() - rotationOffset;
        while (angle < -180){ angle += 360; }
        while (angle > 180){ angle -= 360; }
        return Rotation2d.fromDegrees(angle);
    }

    public void stopModules() {
        modules.forEach((name, module) -> module.stop());
    }

    public Command getTrajectoryCommand(PathPlannerTrajectory trajectory, boolean resetPose) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if(extraTelemetry) field.getObject("traj").setTrajectory(trajectory);
                if(resetPose) {
                    PathPlannerState initialState = trajectory.getInitialState();
                    Pose2d initialPose = initialState.poseMeters;
                    Pose2d startPose = new Pose2d(initialPose.getX(), initialPose.getY(), initialState.holonomicRotation);
                    resetOdometryPose(startPose);
                }
            }),
            new PPSwerveControllerCommand(
                trajectory, this::getPose, kDriveKinematics, 
                xHoldController, yHoldController, thetaHoldControllerAuto, 
                (states) -> setModuleStates(states), this)
        );
        
    }

    public Command getTrajectoryCommand(PathPlannerTrajectory trajectory) {
        return getTrajectoryCommand(trajectory, false);
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public double getGyroHeading() {
        return gyro.getYaw();
    }

    public boolean isFieldRelative() {return fieldRelative;}
    public void setFieldRelative(boolean value) {fieldRelative = value;}

    public void setHoldAngle(Rotation2d angle) {
        holdAngle = angle;
        thetaHoldControllerTele.setSetpoint(angle.getRadians());
    }

    /* RESET COMMANDS FOR DIFFERENT ASPECTS */

    public void resetGyro(Rotation2d angle) {
        gyro.setYaw(angle.getDegrees());
        rotationOffset = 0;
        holdAngle = angle;
    }

    public void resetGyro() {resetGyro(new Rotation2d());}


    public void resetOdometryPose(Pose2d newPose) {
        resetGyro(newPose.getRotation());
        odometry.resetPosition(newPose, newPose.getRotation());
    }

    public void resetOdometryPose() {resetOdometryPose(new Pose2d());}
    
}
