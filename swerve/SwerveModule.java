package frc.robot.ShamLib.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.motors.talonfx.MotionMagicTalonFX;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.talonfx.VelocityTalonFX;
import java.util.function.BooleanSupplier;

public class SwerveModule implements Sendable {

  private final String moduleName;

  private final MotionMagicTalonFX turnMotor;
  private final VelocityTalonFX driveMotor;

  private final CANcoder turnEncoder;
  private final Rotation2d encoderOffset;

  private SwerveModuleState targetState;

  private double targetModuleAngle;

  private final Translation2d moduleOffset;

  public SwerveModule(
      String name,
      String canbus,
      int turnID,
      int driveID,
      int encoderID,
      Rotation2d encoderOffset,
      Translation2d moduleOffset,
      PIDSVGains driveGains,
      PIDSVGains turnGains,
      double maxTurnVelo,
      double maxTurnAccel,
      double turnRatio,
      double driveRatio,
      CurrentLimitsConfigs currentLimit,
      boolean driveInverted,
      boolean turnInverted) {
    this.moduleOffset = moduleOffset;

    this.moduleName = name;

    this.turnEncoder = new CANcoder(encoderID, canbus);
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    this.turnEncoder.getConfigurator().apply(canCoderConfig);

    this.encoderOffset = encoderOffset;

    turnMotor =
        new MotionMagicTalonFX(turnID, canbus, turnGains, turnRatio, maxTurnVelo, maxTurnAccel);
    turnMotor.setInverted(turnInverted); // All turn modules were inverted
    applyCurrentLimit(turnMotor, currentLimit);

    driveMotor = new VelocityTalonFX(driveID, canbus, driveGains, driveRatio);
    driveMotor.setInverted(driveInverted);
    applyCurrentLimit(driveMotor, currentLimit);

    MotorOutputConfigs configs = new MotorOutputConfigs();
    driveMotor.getConfigurator().refresh(configs);
    configs.NeutralMode = NeutralModeValue.Brake;
    driveMotor.getConfigurator().apply(configs);

    pullAbsoluteAngle();

    setDesiredState(new SwerveModuleState(0, getTurnAngle()));
  }

  public SwerveModule(
      String name,
      int turnID,
      int driveID,
      int encoderID,
      Rotation2d encoderOffset,
      Translation2d moduleOffset,
      PIDSVGains driveGains,
      PIDSVGains turnGains,
      double maxTurnVelo,
      double maxTurnAccel,
      double turnRatio,
      double driveRatio,
      CurrentLimitsConfigs currentLimit,
      boolean driveInverted,
      boolean turnInverted) {
    this(
        name,
        "",
        turnID,
        driveID,
        encoderID,
        encoderOffset,
        moduleOffset,
        driveGains,
        turnGains,
        maxTurnVelo,
        maxTurnAccel,
        turnRatio,
        driveRatio,
        currentLimit,
        driveInverted,
        turnInverted);
  }

  private void applyCurrentLimit(TalonFX motor, CurrentLimitsConfigs limit) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    motor.getConfigurator().refresh(config);
    config.CurrentLimits = limit;
    motor.getConfigurator().apply(config);
  }

  private double normalizeDegrees(double degrees) {
    return Math.IEEEremainder(degrees, 360);
  }

  public void setDesiredState(SwerveModuleState state) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getTurnAngle());
    targetState = optimizedState;
    double turnPos = turnMotor.getEncoderPosition();
    targetModuleAngle =
        turnPos + (normalizeDegrees(optimizedState.angle.getDegrees() - normalizeDegrees(turnPos)));

    turnMotor.setTarget(targetModuleAngle);

    driveMotor.setTarget(targetState.speedMetersPerSecond);
  }

  public double getDriveMotorRate() {
    return driveMotor.getEncoderVelocity();
  }

  public double getDriveMotorPosition() {
    return driveMotor.getEncoderPosition();
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
    return turnMotor.calculateKV(kS, 0.05, increment, interrupt);
  }

  public Command calculateDriveKV(
      double kS, Trigger increment, Trigger invert, BooleanSupplier interrupt, boolean telemetry) {
    return driveMotor.calculateKV(kS, 0.05, increment, invert, interrupt, telemetry);
  }

  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromDegrees(
        normalizeDegrees(
            turnEncoder.getAbsolutePosition().getValue() * 360 - encoderOffset.getDegrees()));
  }

  public void pullAbsoluteAngle() {
    turnMotor.resetPosition(
        normalizeDegrees(
            turnEncoder.getAbsolutePosition().getValue() * 360 - encoderOffset.getDegrees()));
  }

  public void resetAngle(Rotation2d angle) {
    turnMotor.resetPosition(angle.getDegrees());
  }

  public Rotation2d getTurnAngle() {
    return Rotation2d.fromDegrees(normalizeDegrees(turnMotor.getEncoderPosition()));
  }

  public double getTurnMotorVelo() {
    return turnMotor.getEncoderVelocity();
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
    builder.addDoubleProperty(
        "Raw encoder angle", () -> turnEncoder.getAbsolutePosition().getValue() * 360, null);
    builder.addDoubleProperty(
        "Angle (wrapped - encoder)",
        () ->
            normalizeDegrees(
                turnEncoder.getAbsolutePosition().getValue() * 360 - encoderOffset.getDegrees()),
        null);
    builder.addDoubleProperty(
        "Position error",
        () -> Math.abs(turnMotor.getTarget() - turnMotor.getEncoderPosition()),
        null);
    builder.addDoubleProperty("Target Angle", () -> targetState.angle.getDegrees(), null);
    builder.addDoubleProperty("Velocity", () -> getDriveMotorRate(), null);
    builder.addDoubleProperty("Target Velocity", () -> targetState.speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Velo error", () -> Math.abs(driveMotor.getTarget() - getDriveMotorRate()), null);
  }
}
