package frc.robot.ShamLib.swerve;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.motors.PIDFGains;

import java.util.List;
import java.util.ArrayList;


public class SwerveDrivetrain extends SubsystemBase {

    protected final List<SwerveModule> modules;
    protected final SwerveDriveKinematics kDriveKinematics;
    protected final double maxChassisSpeed;
    private int numModules = 0;
    private WPI_Pigeon2 gyro;
    private double rotationOffset;
    private Rotation2d holdAngle;

    protected final PIDController thetaHoldControllerTele, thetaHoldControllerAuto, xHoldController, yHoldController;
    private SwerveDrivePoseEstimator odometry;

    private boolean fieldRelative = true;

    protected Field2d field;
    private boolean extraTelemetry;

    //TODO: Fix errors

    /**
     * Constructor for your typical swerve drivetarin with odometry compatible with vision pose estimation
     * @param pigeon2ID CAN idea of the pigeon 2 gyro
     * @param moduleDriveGains PIDF gains for the velocity of the swerve modules
     * @param moduleTurnGains PIDF gains for the position of the swerve modules
     * @param maxModuleTurnVelo maximum velocity the turn motors should go
     * @param maxModuleTurnAccel maximum acceleration the turn motors should go
     * @param teleThetaGains PID gains for the angle hold controller in teleop
     * @param autoThetaGains PID gains for the angle hold controller in autonomous
     * @param translationGains PID gains for the trans
     * @param extraTelemetry whether to send additional telemetry data, like vision pose measurements, trajectory data, and module poses
     * @param moduleCanbus The canbus the modules are on (pass "" for default)
     * @param gyroCanbus The canbus the gyro is on (pass "" for default)
     * @param moduleInfos Array of module infos, one for each module
     */
    public SwerveDrivetrain(int pigeon2ID,
                            PIDFGains moduleDriveGains,
                            PIDFGains moduleTurnGains,
                            double maxChassisSpeed,
                            double maxModuleTurnVelo,
                            double maxModuleTurnAccel,
                            PIDGains teleThetaGains,
                            PIDGains autoThetaGains,
                            PIDGains translationGains,
                            boolean extraTelemetry,
                            String moduleCanbus,
                            String gyroCanbus,
                            SupplyCurrentLimitConfiguration currentLimit,
                            ModuleInfo... moduleInfos) {

        this.extraTelemetry = extraTelemetry;
        this.maxChassisSpeed = maxChassisSpeed;

        //Apply the gains passed in the constructors
        thetaHoldControllerTele =  teleThetaGains.applyToController();

        thetaHoldControllerAuto = autoThetaGains.applyToController();
        xHoldController = translationGains.applyToController();
        yHoldController = translationGains.applyToController();

        gyro = new WPI_Pigeon2(pigeon2ID, gyroCanbus);

        modules = new ArrayList<>();
        Translation2d[] offsets = new Translation2d[moduleInfos.length];
        for(int i = 0; i<moduleInfos.length; i++) {
            numModules++;
            ModuleInfo m = moduleInfos[i];
            offsets[i] = m.offset;

            modules.add(new SwerveModule("Module-" + numModules, moduleCanbus, m.turnMotorID, m.driveMotorID,
                    m.encoderID, m.encoderOffset, m.offset, moduleDriveGains, moduleTurnGains, maxModuleTurnVelo, maxModuleTurnAccel, m.turnRatio, m.driveRatio, currentLimit));
        }

        gyro.configFactoryDefault();

        rotationOffset = getGyroHeading();
        holdAngle = new Rotation2d(rotationOffset);
        thetaHoldControllerTele.setTolerance(Math.toRadians(1.5));

        kDriveKinematics = new SwerveDriveKinematics(offsets);

        odometry = new SwerveDrivePoseEstimator(kDriveKinematics, getCurrentAngle(), getModulePositions(), new Pose2d());

        thetaHoldControllerTele.enableContinuousInput(-Math.PI, Math.PI);
        thetaHoldControllerAuto.enableContinuousInput(-Math.PI, Math.PI);
        field = new Field2d();
    }

        

    public void addVisionMeasurement(Pose2d pose) {
        if (extraTelemetry) field.getObject("vision").setPose(pose);

        odometry.addVisionMeasurement(pose, Timer.getFPGATimestamp());
    }


    /**
     * Updates the odometry pose estimator.
     * THIS MUST BE CALLED PERIODICALLY
     */
    public void updateOdometry() {
        odometry.update(getCurrentAngle(), getModulePositions());
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[numModules];

        for(int i = 0; i < modules.size(); i++) {
            states[i] = modules.get(i).getCurrentState();
        }

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[numModules];

        for(int i = 0; i < modules.size(); i++) {
            positions[i] = modules.get(i).getCurrentPosition();
        }

        return positions;
    }

    /**
     * Should be called periodically if you want the fireld to regularly be updated
     */
    private void updateField2dObject() {
        Pose2d robotPose = getPose();
        field.setRobotPose(robotPose);

        if(extraTelemetry) {
            //Send each of the module poses to the dashboard as well
            for(SwerveModule e : modules) {
                field.getObject(e.getModuleName()).setPose(calculateModulePose(e, robotPose));
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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxChassisSpeed);

        setModuleStates(swerveModuleStates);
    }

    /**
     * Sets the target state of each swerve module based on the input array
     * @param states array of swerve module states
     */
    public void setModuleStates(SwerveModuleState[] states) {
        for(int i = 0; i<states.length; i++) {
            modules.get(i).setDesiredState(states[i]);
        }
    }

    public void setAllModules(SwerveModuleState state) {
        for(SwerveModule module : modules) {
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
        modules.forEach((module) -> module.stop());
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
        odometry.resetPosition(newPose.getRotation(), getModulePositions(), newPose);
    }

    public void resetOdometryPose() {resetOdometryPose(new Pose2d());}
    
}
