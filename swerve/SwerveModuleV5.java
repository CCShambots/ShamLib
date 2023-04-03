package frc.robot.ShamLib.swerve;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShamLib.motors.v5.MotionMagicTalonFXV5;
import frc.robot.ShamLib.motors.v5.PIDFGains;
import frc.robot.ShamLib.motors.v5.VelocityTalonFXV5;

public class SwerveModuleV5 implements Sendable{

    private final String moduleName;

    private final MotionMagicTalonFXV5 turnMotor;
    private final VelocityTalonFXV5 driveMotor;

    private final CANCoder turnEncoder;
    private final double encoderOffset;

    private SwerveModuleState targetState;

    private double targetModuleAngle;

    private final Translation2d moduleOffset;

    public SwerveModuleV5(String name,
                        String canbus,
                        int turnID,
                        int driveID,
                        int encoderID,
                        double encoderOffset,
                        Translation2d moduleOffset,
                        PIDFGains driveGains,
                        PIDFGains turnGains,
                        double maxTurnVelo,
                        double maxTurnAccel,
                        double turnRatio,
                        double driveRatio,
                        SupplyCurrentLimitConfiguration currentLimit,
                        boolean driveInverted,
                        boolean turnInverted
    ) {
        this.moduleOffset = moduleOffset;

        this.moduleName = name;

        this.turnEncoder = new CANCoder(encoderID, canbus);
        turnEncoder.configFactoryDefault();

        turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        turnEncoder.configSensorDirection(false);

        this.encoderOffset = encoderOffset;

        turnMotor = new MotionMagicTalonFXV5(turnID, canbus, turnGains, turnRatio, maxTurnVelo, maxTurnAccel);
        turnMotor.setInverted(turnInverted); //All turn modules were inverted
        turnMotor.configNeutralDeadband(0.01);
        turnMotor.resetPosition(-normalizeDegrees(turnEncoder.getAbsolutePosition() - encoderOffset));
        turnMotor.configSupplyCurrentLimit(currentLimit);

        driveMotor = new VelocityTalonFXV5(driveID, canbus, driveGains, driveRatio);
        driveMotor.setInverted(driveInverted);

        driveMotor.configSupplyCurrentLimit(currentLimit);

        setDesiredState(
                new SwerveModuleState(0, getTurnAngle())
        );
    }

    public SwerveModuleV5(String name,
                        int turnID,
                        int driveID,
                        int encoderID,
                        double encoderOffset,
                        Translation2d moduleOffset,
                        PIDFGains driveGains,
                        PIDFGains turnGains,
                        double maxTurnVelo,
                        double maxTurnAccel,
                        double turnRatio,
                        double driveRatio,
                        SupplyCurrentLimitConfiguration currentLimit,
                        boolean driveInverted,
                        boolean turnInverted
    ) {
        this(name, "", turnID, driveID, encoderID, encoderOffset, moduleOffset, driveGains, turnGains, maxTurnVelo, maxTurnAccel, turnRatio, driveRatio, currentLimit, driveInverted, turnInverted);
    }

    public double getAbsoluteAngle() {
        return turnEncoder.getAbsolutePosition();
    }

    private double normalizeDegrees(double degrees) {
        return Math.IEEEremainder(degrees, 360);
    }

    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getTurnAngle());
        targetState = optimizedState;
        double turnPos = turnMotor.getPosition();
        targetModuleAngle = turnPos + (normalizeDegrees(optimizedState.angle.getDegrees() - normalizeDegrees(turnPos)));

        turnMotor.setTarget(targetModuleAngle);

        driveMotor.setTarget(targetState.speedMetersPerSecond);
    }

    public Rotation2d getTurnAngle(){
        return Rotation2d.fromDegrees(normalizeDegrees(turnMotor.getPosition()));
    }

    public double getDriveMotorRate(){
        return driveMotor.getVelocity();
    }

    public double getDriveMotorPosition() {
        return driveMotor.getPosition();
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDriveMotorRate(), getTurnAngle());
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(getDriveMotorPosition(), getTurnAngle());
    }

    public Translation2d getModuleOffset() {return moduleOffset;}

    public void stop() {
        setDesiredState(new SwerveModuleState(0.0, getTurnAngle()));
    }

    public String getModuleName() {
        return moduleName;
    }

    public Command calculateTurnKf(BooleanSupplier interrupt) {
        return driveMotor.calculateKF(1, interrupt);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Module");

        builder.addDoubleProperty("Angle", () -> getTurnAngle().getDegrees(), null);
        builder.addDoubleProperty("Angle (wrapped - encoder)", () -> normalizeDegrees(turnEncoder.getAbsolutePosition() - encoderOffset), null);
        // builder.addDoubleProperty("Degree Angle (motor)", () -> turnMotor.getPosition(), null);
        // builder.addDoubleProperty("Raw Angle (motor)", () -> turnMotor.getSelectedSensorPosition(), null);
        builder.addDoubleProperty("Raw Setpoint", () -> driveMotor.getClosedLoopTarget(0), null);
        builder.addDoubleProperty("erorr", () -> Math.abs(turnMotor.getTarget() - turnMotor.getPosition()), null);
        // builder.addDoubleProperty("Absolute Angle", () -> turnEncoder.getAbsolutePosition(), null);
        builder.addDoubleProperty("Target Angle", () -> targetState.angle.getDegrees(), null);
        builder.addDoubleProperty("Velocity", () -> getDriveMotorRate(), null);
        builder.addDoubleProperty("Target Velocity", () -> targetState.speedMetersPerSecond, null);
        builder.addDoubleProperty("Velo error", () -> Math.abs(driveMotor.getTarget() - driveMotor.getVelocity()), null);

        // builder.addDoubleProperty("Encoder offset", () -> encoderOffset, null);
    }

}