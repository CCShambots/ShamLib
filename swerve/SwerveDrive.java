package frc.robot.ShamLib.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.ShamLibConstants.BuildMode;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.swerve.gyro.GyroIO;
import frc.robot.ShamLib.swerve.gyro.GyroIOReal;
import frc.robot.ShamLib.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.ShamLib.swerve.module.ModuleInfo;
import frc.robot.ShamLib.swerve.module.SwerveModule;
import frc.robot.ShamLib.swerve.module.SwerveModuleIO;
import frc.robot.ShamLib.swerve.module.SwerveModuleIOReal;
import frc.robot.ShamLib.swerve.module.SwerveModuleIOSim;
import frc.robot.ShamLib.swerve.odometry.SwerveOdometry;
import frc.robot.ShamLib.swerve.odometry.SwerveOdometryReal;
import frc.robot.ShamLib.swerve.odometry.SwerveOdometrySim;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive {

  protected final List<SwerveModule> modules;

  protected final GyroIO gyroIO;
  protected final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();

  private final BuildMode buildMode;

  protected final SwerveOdometry odometry;

  protected final SwerveDriveKinematics kDriveKinematics;
  protected final double maxChassisSpeed;
  protected final double maxChassisAcceleration;
  protected final double maxChassisRotationVel;
  protected final double maxChassisRotationAccel;
  private int numModules = 0;
  private Rotation2d rotationOffset;
  private Rotation2d holdAngle;

  private boolean fieldRelative = true;

  private final Field2d field;
  private final boolean extraTelemetry;

  private int speedMode = 0;

  private final PIDGains translationGains;
  private final PIDGains rotationGains;
  private final double driveBaseRadius; // In meters

  // Pose used for simulation
  private Pose2d simPose;

  /**
   * Constructor for your typical swerve drive with odometry compatible with vision pose estimation
   *
   * @param pigeon2ID CAN idea of the pigeon 2 gyro
   * @param moduleDriveGains PIDSV gains for the velocity of the swerve modules
   * @param moduleTurnGains PIDSV gains for the position of the swerve modules
   * @param maxModuleTurnVelo maximum velocity the turn motors should go
   * @param maxModuleTurnAccel maximum acceleration the turn motors should go
   * @param teleThetaGains PID gains for the angle hold controller in teleop
   * @param autoThetaGains PID gains for the angle hold controller in autonomous
   * @param translationGains PID gains for the trans
   * @param extraTelemetry whether to send additional telemetry data, like vision pose measurements,
   *     trajectory data, and module poses
   * @param moduleCanbus The canbus the modules are on (pass "" for default)
   * @param gyroCanbus The canbus the gyro is on (pass "" for default)
   * @param moduleInfos Array of module infos, one for each module
   */
  public SwerveDrive(
      BuildMode mode,
      int pigeon2ID,
      PIDSVGains moduleDriveGains,
      PIDSVGains moduleTurnGains,
      double maxChassisSpeed,
      double maxChassisAccel,
      double maxChassisRotationVel,
      double maxChassisRotationAccel,
      double maxModuleTurnVelo,
      double maxModuleTurnAccel,
      PIDGains autoThetaGains,
      PIDGains translationGains,
      boolean extraTelemetry,
      String moduleCanbus,
      String gyroCanbus,
      CurrentLimitsConfigs currentLimit,
      Subsystem subsystem,
      ModuleInfo... moduleInfos) {

    this.buildMode = mode;

    this.extraTelemetry = extraTelemetry;
    this.maxChassisSpeed = maxChassisSpeed;
    this.maxChassisAcceleration = maxChassisAccel;
    this.maxChassisRotationVel = maxChassisRotationVel;
    this.maxChassisRotationAccel = maxChassisRotationAccel;

    this.translationGains = translationGains;
    this.rotationGains = autoThetaGains;

    modules = new ArrayList<>();
    Translation2d[] offsets = new Translation2d[moduleInfos.length];
    for (int i = 0; i < moduleInfos.length; i++) {
      numModules++;
      ModuleInfo m = moduleInfos[i];
      offsets[i] = m.offset;

      SwerveModuleIO io = new SwerveModuleIO() {};

      switch (mode) {
        case REAL:
          io =
              new SwerveModuleIOReal(
                  gyroCanbus,
                  m,
                  moduleDriveGains,
                  moduleTurnGains,
                  maxModuleTurnVelo,
                  maxModuleTurnAccel,
                  currentLimit);
          break;
        case SIM:
          io =
              new SwerveModuleIOSim(
                  gyroCanbus,
                  m,
                  moduleDriveGains,
                  moduleTurnGains,
                  maxModuleTurnVelo,
                  maxModuleTurnAccel,
                  currentLimit);
        default:
          break;
      }

      modules.add(
          new SwerveModule(
              io,
              "Module-" + numModules,
              moduleCanbus,
              m,
              moduleDriveGains,
              moduleTurnGains,
              maxModuleTurnVelo,
              maxModuleTurnAccel));

      if (extraTelemetry) {
        SmartDashboard.putData("Module-" + i, modules.get(i));
      }
    }

    kDriveKinematics = new SwerveDriveKinematics(offsets);

    switch (mode) {
      case REAL:
        gyroIO = new GyroIOReal(pigeon2ID, gyroCanbus);
        odometry =
            new SwerveOdometryReal(
                new SwerveDrivePoseEstimator(
                    kDriveKinematics, getCurrentAngle(), getModulePositions(), new Pose2d()));
        break;

      default:
        odometry = new SwerveOdometrySim(kDriveKinematics, modules);
        gyroIO = new GyroIO() {};
        break;
    }

    rotationOffset = getGyroHeading();
    holdAngle = new Rotation2d(rotationOffset.getRadians());

    field = new Field2d();

    this.driveBaseRadius =
        Math.hypot(
            moduleInfos[0].offset.getX(),
            moduleInfos[0].offset.getY()); // Radius of the drive base in meters

    // Configure the auto builder stuff
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometryPose,
        this::getChassisSpeeds,
        this::drive,
        new HolonomicPathFollowerConfig(
            translationGains.toPIDConstants(),
            autoThetaGains.toPIDConstants(),
            maxChassisSpeed,
            driveBaseRadius,
            new ReplanningConfig()),
        subsystem);
  }

  /*MUST BE CALLED PERIODICALLY */
  public void update() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Gyro", gyroInputs);

    for (SwerveModule m : modules) {
      m.update();
    }

    updateOdometry();
  }

  public Rotation2d getPitch() {
    return gyroInputs.gyroPitch;
  }

  public Rotation2d getRoll() {
    return gyroInputs.gyroRoll;
  }

  public void addVisionMeasurement(Pose2d pose) {
    if (extraTelemetry) field.getObject("vision").setPose(pose);

    odometry.addVisionMeasurement(pose);
  }

  /** Updates the odometry pose estimator. THIS MUST BE CALLED PERIODICALLY */
  private void updateOdometry() {
    switch (buildMode) {
      case SIM:
        // Update the odometry for sim (everything it needs is already passed in in the constructor)
        odometry.updatePose();
        break;
      default:
        // Update odometry normally if being fed data from a real robot
        odometry.updatePose(getCurrentAngle(), getModulePositions());
        break;
    }
  }

  public double[] getModuleAngles() {

    double[] angles = new double[numModules];

    for (int i = 0; i < modules.size(); i++) {
      angles[i] = modules.get(i).getCurrentState().angle.getDegrees();
    }

    return angles;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[numModules];

    for (int i = 0; i < modules.size(); i++) {
      states[i] = modules.get(i).getCurrentState();
    }

    return states;
  }

  public SwerveModuleState[] getTargetModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[numModules];

    for (int i = 0; i < modules.size(); i++) {
      states[i] = modules.get(i).getTargetState();
    }

    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[numModules];

    for (int i = 0; i < modules.size(); i++) {
      positions[i] = modules.get(i).getCurrentPosition();
    }

    return positions;
  }

  /** Should be called periodically if you want the field to regularly be updated */
  public void updateField2dObject() {
    Pose2d robotPose = getPose();
    field.setRobotPose(robotPose);

    if (extraTelemetry) {
      // Send each of the module poses to the dashboard as well
      for (SwerveModule e : modules) {
        field.getObject(e.getModuleName()).setPose(calculateModulePose(e, robotPose));
      }
    }
  }

  public double[] getModuleAbsoluteAngles() {
    double[] out = new double[4];

    for (int i = 0; i < modules.size(); i++) {
      out[i] = modules.get(i).getAbsoluteAngle().getDegrees();
    }

    return out;
  }

  private Pose2d calculateModulePose(SwerveModule module, Pose2d robotPose) {
    SwerveModuleState state = module.getCurrentState();
    Translation2d offset = module.getModuleOffset();

    return new Pose2d(
        robotPose.getTranslation().plus(offset), robotPose.getRotation().plus(state.angle));
  }

  /**
   * Method to call to update the states of the swerve drivetrain
   *
   * @param speeds chassis speed object to move
   */
  public void drive(ChassisSpeeds speeds, double maxChassisSpeed) {

    SwerveModuleState[] swerveModuleStates = kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxChassisSpeed);

    setModuleStates(swerveModuleStates);
  }

  public void drive(ChassisSpeeds speeds) {
    drive(speeds, maxChassisSpeed);
  }

  /**
   * Sets the target state of each swerve module based on the input array
   *
   * @param states array of swerve module states
   */
  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      modules.get(i).setDesiredState(states[i]);
    }
  }

  public void setAllModules(SwerveModuleState state) {
    for (SwerveModule module : modules) {
      module.setDesiredState(state);
    }
  }

  /**
   * Finds the angle of the robot in radians (limited -PI to PI)
   *
   * @return Robot angle
   */
  public Rotation2d getCurrentAngle() {
    // return new Rotation2d(Math.IEEEremainder((getGyroHeading() - rotationOffset) * (Math.PI/180),
    // Math.PI * 2));
    return getGyroHeading().minus(rotationOffset);
  }

  public void stopModules() {
    modules.forEach(SwerveModule::stop);
  }

  /**
   * Get a command to run a path-planner trajectory on the swerve drive
   *
   * @param path the trajectory to run
   * @param resetPose whether to being the command by resetting the pose of the robot
   * @param requirements the subsystem you may need
   * @return the command to run
   */
  public Command getPathCommand(
      PathPlannerPath path, boolean resetPose, Subsystem... requirements) {
    Consumer<SwerveModuleState[]> test = a -> setModuleStates(a);
    // TODO: Fix max speeds
    new SwerveControllerCommand(null, null, kDriveKinematics, null, test, requirements);
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // TODO: Figure out how to post
              // if(extraTelemetry) field.getObject("traj").setTrajectory(path);
              if (resetPose) {
                Pose2d startPose = path.getPreviewStartingHolonomicPose();
                resetOdometryPose(startPose);
              }
            }),
        new FollowPathHolonomic(
            path,
            this::getPose,
            this::getChassisSpeeds,
            this::drive,
            translationGains.toPIDConstants(),
            rotationGains.toPIDConstants(),
            maxChassisSpeed,
            driveBaseRadius,
            new ReplanningConfig(),
            requirements));
  }

  public Command getTrajectoryCommand(PathPlannerPath trajectory, Subsystem... requirements) {
    return getPathCommand(trajectory, false, requirements);
  }

  public Pose2d getPose() {
    return odometry.getPose();
  }

  public Rotation2d getGyroHeading() {
    return gyroInputs.gyroYaw;
  }

  public Rotation2d getHoldAngle() {
    return holdAngle;
  }

  public boolean isFieldRelative() {
    return fieldRelative;
  }

  public void setFieldRelative(boolean value) {
    fieldRelative = value;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public ChassisSpeeds getTargetChassisSpeeds() {
    return kDriveKinematics.toChassisSpeeds(getTargetModuleStates());
  }

  public int getSpeedMode() {
    return speedMode;
  }

  public void setSpeedMode(int speedMode) {
    this.speedMode = speedMode;
  }

  /* RESET COMMANDS FOR DIFFERENT ASPECTS */

  // TODO: Make this play nice with the odometry. Need to test if this is even an issue
  public void resetGyro(Rotation2d angle) {
    gyroIO.setGyroYaw(angle);
    rotationOffset = new Rotation2d();
    holdAngle = angle;
  }

  public void resetRotationOffset(Rotation2d angle) {
    rotationOffset = angle;
  }

  public void resetGyro() {
    resetGyro(new Rotation2d());
  }

  public void resetOdometryPose(Pose2d newPose) {
    resetGyro(newPose.getRotation());
    odometry.resetPose(newPose, getModulePositions());
  }

  public void resetOdometryPose() {
    resetOdometryPose(new Pose2d());
  }

  public Field2d getField() {
    return field;
  }

  public List<SwerveModule> getModules() {
    return modules;
  }

  public Command calculateTurnKV(double kS, Trigger increment, BooleanSupplier interrupt) {
    return modules.get(0).calculateTurnKV(kS, increment, interrupt);
  }

  public Command calculateDriveKV(
      double kS, Trigger increment, Trigger invert, BooleanSupplier interrupt) {
    Command toRun = new InstantCommand();
    boolean first = true;
    for (SwerveModule module : modules) {
      toRun = toRun.alongWith(module.calculateDriveKV(kS, increment, invert, interrupt, first));
      if (first) first = false;
    }

    return toRun;
  }
}
