package frc.robot.ShamLib.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;

public class SwerveModule implements Sendable{

    private final String moduleName;

    private final WPI_TalonFX turnMotor;
    private final WPI_TalonFX driveMotor;

    private final CANCoder turnEncoder;
    private double encoderOffset;

    private SwerveModuleState targetState;

    private double targetEncoderPos;

    private Translation2d moduleOffset;

    public SwerveModule(String name, int turnID, int driveID, int encoderID, double encoderOffset, Translation2d moduleOffset) {
        this.moduleOffset = moduleOffset;
        
        this.moduleName = name;
        turnMotor = new WPI_TalonFX(turnID, "Drivetrain");
        turnMotor.configFactoryDefault();
        turnMotor.setInverted(true); //All turn modules were inverted
        
        driveMotor = new WPI_TalonFX(driveID, "Drivetrain");
        driveMotor.configFactoryDefault();

        this.turnEncoder = new CANCoder(encoderID);
        turnEncoder.configFactoryDefault();

        turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        turnEncoder.configSensorDirection(false);

        this.encoderOffset = encoderOffset;

        initTurnMotor();
        initDriveMotor();

        setDesiredState(
            new SwerveModuleState(0, getTurnAngle())
        );
    }   
        
    private void initTurnMotor() {
        turnMotor.setSensorPhase(false);
        turnMotor.configSupplyCurrentLimit(Constants.CURRENT_LIMIT);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        turnMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.SwerveModule.kTimeoutMs);
        turnMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.SwerveModule.kTimeoutMs);

        /* Set the peak and nominal outputs */
        turnMotor.configNominalOutputForward(0, Constants.SwerveModule.kTimeoutMs);
        turnMotor.configNominalOutputReverse(0, Constants.SwerveModule.kTimeoutMs);
        turnMotor.configPeakOutputForward(1, Constants.SwerveModule.kTimeoutMs);
        turnMotor.configPeakOutputReverse(-1, Constants.SwerveModule.kTimeoutMs);

        /* Set Motion Magic gains in slot0 - see documentation */
        turnMotor.selectProfileSlot(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.kPIDLoopIdx);
        turnMotor.config_kF(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.turnGains.kF, Constants.SwerveModule.kTimeoutMs);
        turnMotor.config_kP(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.turnGains.kP, Constants.SwerveModule.kTimeoutMs);
        turnMotor.config_kI(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.turnGains.kI, Constants.SwerveModule.kTimeoutMs);
        turnMotor.config_kD(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.turnGains.kD, Constants.SwerveModule.kTimeoutMs);

        /* Set acceleration and cruise velocity - see documentation */
        turnMotor.configMotionCruiseVelocity(
                Math.toDegrees(Constants.SwerveModule.MAX_TURN_SPEED),
                Constants.SwerveModule.kTimeoutMs
        );

        turnMotor.configMotionAcceleration(
                Math.toDegrees(Constants.SwerveModule.MAX_TURN_ACCEL),
                Constants.SwerveModule.kTimeoutMs
        );

        turnMotor.configNeutralDeadband(0.001);

        turnMotor.configSelectedFeedbackSensor(
                TalonFXFeedbackDevice.IntegratedSensor,
                /*TODO: add constant*/0,
                /*TODO: add constant*/30
        );

        turnMotor.configSelectedFeedbackCoefficient(
                Constants.SwerveModule.TURN_SENSOR_RATIO * (360.0/2048.0)
        );
        turnMotor.setSelectedSensorPosition(normalizeDegrees(turnEncoder.getAbsolutePosition() - encoderOffset));
    }

    private void initDriveMotor() {
        driveMotor.setSensorPhase(false);
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

        driveMotor.configSupplyCurrentLimit(Constants.CURRENT_LIMIT);

        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.SwerveModule.kTimeoutMs);

        driveMotor.configNominalOutputForward(0, Constants.SwerveModule.kTimeoutMs);
        driveMotor.configNominalOutputReverse(0, Constants.SwerveModule.kTimeoutMs);
        driveMotor.configPeakOutputForward(1, Constants.SwerveModule.kTimeoutMs);
        driveMotor.configPeakOutputReverse(-1, Constants.SwerveModule.kTimeoutMs);

        driveMotor.selectProfileSlot(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.kPIDLoopIdx);
        driveMotor.config_kF(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.driveGains.kF, Constants.SwerveModule.kTimeoutMs);
        driveMotor.config_kP(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.driveGains.kP, Constants.SwerveModule.kTimeoutMs);
        driveMotor.config_kI(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.driveGains.kI, Constants.SwerveModule.kTimeoutMs);
        driveMotor.config_kD(Constants.SwerveModule.kSlotIdx, Constants.SwerveModule.driveGains.kD, Constants.SwerveModule.kTimeoutMs);



    }

    private double driveTicksToMeters(double ticks) {
        return ticks 
        / 2048.0
        * (1 / Constants.SwerveModule.DRIVE_RATIO)
        * (2 * Math.PI * Constants.SwerveModule.WHEEL_RADIUS);
    }

    private double driveMetersToTicks(double meters) {
        return meters
        / (2 * Math.PI * Constants.SwerveModule.WHEEL_RADIUS)
        / (1 / Constants.SwerveModule.DRIVE_RATIO)
        * 2048;
    }


    private double normalizeDegrees(double degrees) {
        double rads = Math.toRadians(degrees);
        //TODO: I might be broken, probably not but you never know
        return Math.toDegrees(Math.atan2(Math.sin(rads), Math.cos(rads)));
    }

    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getTurnAngle());
        targetState = optimizedState;        
        double turnPos = turnMotor.getSelectedSensorPosition();
        targetEncoderPos = turnPos + (normalizeDegrees(optimizedState.angle.getDegrees() - normalizeDegrees(turnPos)));
        
        turnMotor.set(
                ControlMode.MotionMagic,
                targetEncoderPos
                // DemandType.ArbitraryFeedForward,
                // Constants.SwerveModule.KS_TURN/12
        );

        driveMotor.set(
            ControlMode.Velocity,
            driveMetersToTicks(targetState.speedMetersPerSecond)/10
            // DemandType.ArbitraryFeedForward,
            // Constants.SwerveModule.KS_DRIVE/12
        );

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
        //TODO: add these but for ctre pid
        //builder.addDoubleProperty("Target turn velo", () -> turnPIDController.getSetpoint().velocity, null);
        //builder.addDoubleProperty("Measuerd turn velo", () -> reverseTurnEncoder ? -1 : 1 * turnEncoder.getVelocity(), null);
        
    }
    
}
