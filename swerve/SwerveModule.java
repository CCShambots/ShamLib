package frc.robot.ShamLib.swerve;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.ShamLib.motors.MotionMagicTalonFX;
import frc.robot.ShamLib.motors.PIDFGains;
import frc.robot.ShamLib.motors.VelocityTalonFX;

public class SwerveModule implements Sendable{

    private final String moduleName;

    private final MotionMagicTalonFX turnMotor;
    private final VelocityTalonFX driveMotor;

    private final CANCoder turnEncoder;
    private double encoderOffset;

    private SwerveModuleState targetState;

    private double targetModuleAngle;

    private Translation2d moduleOffset;

    public SwerveModule(String name,
                        int turnID,
                        int driveID,
                        String canbus,
                        int encoderID,
                        double encoderOffset,
                        Translation2d moduleOffset,
                        PIDFGains driveGains,
                        PIDFGains turnGains,
                        double maxTurnVelo,
                        double maxTurnAccel,
                        double turnRatio,
                        double driveRatio
    ) {
        this.moduleOffset = moduleOffset;
        
        this.moduleName = name;

        this.turnEncoder = new CANCoder(encoderID);
        turnEncoder.configFactoryDefault();

        turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        turnEncoder.configSensorDirection(false);

        this.encoderOffset = encoderOffset;

        turnMotor = new MotionMagicTalonFX(turnID, canbus, turnGains, turnRatio, maxTurnVelo, maxTurnAccel);
        turnMotor.setInverted(true); //All turn modules were inverted
        turnMotor.configNeutralDeadband(0.01);
        turnMotor.resetPosition(normalizeDegrees(turnEncoder.getAbsolutePosition() - encoderOffset));

        driveMotor = new VelocityTalonFX(driveID, canbus, driveGains, driveRatio);
        driveMotor.configFactoryDefault();

        driveMotor.setSensorPhase(false);
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        driveMotor.configSupplyCurrentLimit(Constants.CURRENT_LIMIT);

        setDesiredState(
            new SwerveModuleState(0, getTurnAngle())
        );
    }

    private double normalizeDegrees(double degrees) {
        double rads = Math.toRadians(degrees);
        return Math.toDegrees(Math.atan2(Math.sin(rads), Math.cos(rads)));
    }

    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getTurnAngle());
        targetState = optimizedState;        
        double turnPos = turnMotor.getSelectedSensorPosition();
        targetModuleAngle = turnPos + (normalizeDegrees(optimizedState.angle.getDegrees() - normalizeDegrees(turnPos)));

        turnMotor.setTarget(targetModuleAngle);

        driveMotor.setTarget(targetState.speedMetersPerSecond);

    }

    public Rotation2d getTurnAngle(){
        return Rotation2d.fromDegrees(normalizeDegrees(turnMotor.getSelectedSensorPosition()));
    }

    public double getDriveMotorRate(){
        return driveTicksToMeters(driveMotor.getSelectedSensorVelocity()) * 10.0;
    } 

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDriveMotorRate(), getTurnAngle());
    }

    public Translation2d getModuleOffset() {return moduleOffset;}

    public void stop() {
        setDesiredState(new SwerveModuleState(0.0, getTurnAngle()));
    }

    public String getModuleName() {
        return moduleName;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Module");

        builder.addDoubleProperty("Angle", () -> getTurnAngle().getDegrees(), null);
        builder.addDoubleProperty("Raw Angle", () -> turnMotor.getSelectedSensorPosition(), null);
        builder.addDoubleProperty("Raw Setpoint", () -> turnMotor.getClosedLoopTarget(0), null);
        builder.addDoubleProperty("Absolute Angle", () -> turnEncoder.getAbsolutePosition(), null);
        builder.addDoubleProperty("Target Angle", () -> targetState.angle.getDegrees(), null);
        builder.addDoubleProperty("Velocity", () -> getDriveMotorRate(), null);
        builder.addDoubleProperty("Target Velocity", () -> targetState.speedMetersPerSecond, null);
        builder.addDoubleProperty("Encoder offset", () -> encoderOffset, null);
    }
    
}
