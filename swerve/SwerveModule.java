package frc.robot.ShamLib.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.motors.pro.PIDSVGains;
import frc.robot.ShamLib.motors.pro.MotionMagicTalonFXPro;
import frc.robot.ShamLib.motors.pro.VelocityTalonFXPro;

import static frc.robot.ShamLib.ShamLibConstants.Swerve.ALLOWED_STOPPED_MODULE_DIFF;

public class SwerveModule implements Sendable{

    private final String moduleName;

    private final MotionMagicTalonFXPro turnMotor;
    private final VelocityTalonFXPro driveMotor;

    private final CANCoder turnEncoder;
    private double encoderOffset;

    private SwerveModuleState targetState;

    private double targetModuleAngle;

    private Translation2d moduleOffset;

    public SwerveModule(String name,
                        String canbus,
                        int turnID,
                        int driveID,
                        int encoderID,
                        double encoderOffset,
                        Translation2d moduleOffset,
                        PIDSVGains driveGains,
                        PIDSVGains turnGains,
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

        turnMotor = new MotionMagicTalonFXPro(turnID, canbus, turnGains, turnRatio, maxTurnVelo, maxTurnAccel);
        turnMotor.setInverted(turnInverted); //All turn modules were inverted
        pullAbsoluteAngle();

        //TODO: Figure out current limiting

        driveMotor = new VelocityTalonFXPro(driveID, canbus, driveGains, driveRatio);
        driveMotor.setInverted(driveInverted);

        setDesiredState(
            new SwerveModuleState(0, getTurnAngle())
        );
    }

    public SwerveModule(String name,
                        int turnID,
                        int driveID,
                        int encoderID,
                        double encoderOffset,
                        Translation2d moduleOffset,
                        PIDSVGains driveGains,
                        PIDSVGains turnGains,
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


    private double normalizeDegrees(double degrees) {
        return Math.IEEEremainder(degrees, 360);
    }

    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getTurnAngle());
        targetState = optimizedState;        
        double turnPos = turnMotor.getEncoderPosition();
        targetModuleAngle = turnPos + (normalizeDegrees(optimizedState.angle.getDegrees() - normalizeDegrees(turnPos)));

        turnMotor.setTarget(targetModuleAngle);

        driveMotor.setTarget(targetState.speedMetersPerSecond);
    }

    public double getDriveMotorRate(){
        return driveMotor.getEncoderVelocity();
    } 

    public double getDriveMotorPosition() {
        return driveMotor.getEncoderPosition();
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

    public Command calculateTurnKV(double kS, Trigger increment, BooleanSupplier interrupt) {
        return turnMotor.calculateKV(kS, 0.05, increment, interrupt);
    }

    public Command calculateDriveKV(double kS, Trigger increment, Trigger invert, BooleanSupplier interrupt, boolean telemetry) {
        return driveMotor.calculateKV(kS, 0.05, increment, invert, interrupt, telemetry);
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromDegrees(normalizeDegrees(turnEncoder.getAbsolutePosition() - encoderOffset));
    }

    public void pullAbsoluteAngle() {
        turnMotor.resetPosition(normalizeDegrees(turnEncoder.getAbsolutePosition() - encoderOffset));
    }

    public void resetAngle(Rotation2d angle) {
        turnMotor.resetPosition(angle.getDegrees());
    }

    public Rotation2d getTurnAngle(){
        return Rotation2d.fromDegrees(normalizeDegrees(turnMotor.getEncoderPosition()));
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
        builder.addDoubleProperty("Angle (wrapped - encoder)", () -> normalizeDegrees(turnEncoder.getAbsolutePosition() - encoderOffset), null);
        // builder.addDoubleProperty("Degree Angle (motor)", () -> turnMotor.getPosition(), null);
        // builder.addDoubleProperty("Raw Angle (motor)", () -> turnMotor.getSelectedSensorPosition(), null);
        builder.addDoubleProperty("Raw Setpoint", () -> driveMotor.getTarget(), null);
        builder.addDoubleProperty("erorr", () -> Math.abs(turnMotor.getTarget() - turnMotor.getEncoderPosition()), null);
        // builder.addDoubleProperty("Absolute Angle", () -> turnEncoder.getAbsolutePosition(), null);
        builder.addDoubleProperty("Target Angle", () -> targetState.angle.getDegrees(), null);
        builder.addDoubleProperty("Velocity", () -> getDriveMotorRate(), null);
        builder.addDoubleProperty("Target Velocity", () -> targetState.speedMetersPerSecond, null);
        builder.addDoubleProperty("Velo error", () -> Math.abs(driveMotor.getTarget() - getDriveMotorRate()), null);

    }
    
}
