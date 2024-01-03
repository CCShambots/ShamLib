package frc.robot.ShamLib.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class SwerveModule implements Sendable {

  private final SwerveModuleIO io;
  private final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();

  private final String moduleName;

  private final Rotation2d encoderOffset;

  private SwerveModuleState targetState;

  private double targetModuleAngle;

  private final Translation2d moduleOffset;

  private double lastPositionMeters = 0.0; // Used for delta calculation

  public SwerveModule(
      SwerveModuleIO io,
      String name,
      String canbus,
      ModuleInfo moduleInfo,
      PIDSVGains driveGains,
      PIDSVGains turnGains,
      double maxTurnVelo,
      double maxTurnAccel) {

    this.io = io;

    this.moduleOffset = moduleInfo.offset;

    this.moduleName = name;

    this.encoderOffset = moduleInfo.encoderOffset;

    io.updateInputs(inputs);

    pullAbsoluteAngle();

    setDesiredState(new SwerveModuleState(0, getTurnAngle()));
  }

  public SwerveModule(
      SwerveModuleIO io,
      String name,
      ModuleInfo info,
      PIDSVGains driveGains,
      PIDSVGains turnGains,
      double maxTurnVelo,
      double maxTurnAccel) {
    this(io, name, "", info, driveGains, turnGains, maxTurnVelo, maxTurnAccel);
  }

  public void update() {
    io.updateInputs(inputs);
    Logger.processInputs(moduleName, inputs);
  }

  private double normalizeDegrees(double degrees) {
    return Math.IEEEremainder(degrees, 360);
  }

  public void setDesiredState(SwerveModuleState state) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getTurnAngle());
    targetState = optimizedState;
    double turnPos = inputs.turnMotorAngle;
    targetModuleAngle =
        turnPos + (normalizeDegrees(optimizedState.angle.getDegrees() - normalizeDegrees(turnPos)));

    io.setTurnMotorTarget(targetModuleAngle);

    io.setDriveMotorTarget(targetState.speedMetersPerSecond);
  }

  public double getDriveMotorRate() {
    return inputs.driveMotorVelocity;
  }

  public double getDriveMotorPosition() {
    return inputs.driveMotorPosition;
  }

  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(getDriveMotorRate(), getTurnAngle());
  }

  public SwerveModuleState getTargetState() {
    return targetState;
  }

  public SwerveModulePosition getCurrentPosition() {
    return new SwerveModulePosition(getDriveMotorPosition(), getTurnAngle());
  }

  public Translation2d getModuleOffset() {
    return moduleOffset;
  }

  public void stop() {
    setDesiredState(new SwerveModuleState(0.0, getTurnAngle()));
  }

  public String getModuleName() {
    return moduleName;
  }

  public Command calculateTurnKV(double kS, Trigger increment, BooleanSupplier interrupt) {
    return io.calculateTurnKV(kS, increment, interrupt);
  }

  public Command calculateDriveKV(
      double kS, Trigger increment, Trigger invert, BooleanSupplier interrupt, boolean telemetry) {
    return io.calculateDriveKV(kS, increment, invert, interrupt, telemetry);
  }

  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromDegrees(
        normalizeDegrees(inputs.turnEncoderPos - encoderOffset.getDegrees()));
  }

  public void pullAbsoluteAngle() {
    io.resetTurnMotorPosition(normalizeDegrees(inputs.turnEncoderPos - encoderOffset.getDegrees()));
  }

  public void resetAngle(Rotation2d angle) {
    io.resetTurnMotorPosition(angle.getDegrees());
  }

  public Rotation2d getTurnAngle() {
    return Rotation2d.fromDegrees(normalizeDegrees(inputs.turnMotorAngle));
  }

  public double getTurnMotorVelo() {
    return inputs.turnMotorVelocity;
  }

  /** Returns the module position delta since the last call to this method. */
  public SwerveModulePosition getPositionDelta() {
    var delta =
        new SwerveModulePosition(
            getCurrentPosition().distanceMeters - lastPositionMeters, getTurnAngle());
    lastPositionMeters = getCurrentPosition().distanceMeters;
    return delta;
  }

  /**
   * @return the error of the motor position vs the absolute encoder position (in degrees)
   */
  public double getAbsoluteError() {
    return Math.abs(getAbsoluteAngle().minus(getTurnAngle()).getDegrees());
  }

  public boolean isModuleMisaligned() {
    return getAbsoluteError() > ShamLibConstants.Swerve.ALLOWED_MODULE_ERROR;
  }

  public Command realignModule() {
    return new RealignModuleCommand(this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve Module");

    builder.addDoubleProperty("Angle", () -> getTurnAngle().getDegrees(), null);
    builder.addDoubleProperty("Raw encoder angle", () -> inputs.turnEncoderPos, null);
    builder.addDoubleProperty(
        "Angle (wrapped - encoder)",
        () -> normalizeDegrees(inputs.turnEncoderPos - encoderOffset.getDegrees()),
        null);
    builder.addDoubleProperty(
        "Position error", () -> Math.abs(inputs.turnMotorTarget - inputs.turnMotorAngle), null);
    builder.addDoubleProperty("Target Angle", () -> targetState.angle.getDegrees(), null);
    builder.addDoubleProperty("Velocity", () -> getDriveMotorRate(), null);
    builder.addDoubleProperty("Target Velocity", () -> targetState.speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Velo error", () -> Math.abs(inputs.driveMotorTarget - getDriveMotorRate()), null);
  }
}
