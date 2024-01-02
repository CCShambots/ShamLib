package frc.robot.ShamLib.swerve.module;


import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.motors.talonfx.MotionMagicTalonFX;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.talonfx.VelocityTalonFX;

public class SwerveModuleIOReal implements SwerveModuleIO{
    
  protected final MotionMagicTalonFX turnMotor;
  protected final VelocityTalonFX driveMotor;

  private final CANcoder turnEncoder;

  public SwerveModuleIOReal(
    String canbus,
    ModuleInfo mI,
    PIDSVGains driveGains,
    PIDSVGains turnGains,
    double maxTurnVelo,
    double maxTurnAccel,
    CurrentLimitsConfigs currentLimit
  ) {

    this.turnEncoder = new CANcoder(mI.encoderID, canbus);
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    this.turnEncoder.getConfigurator().apply(canCoderConfig);

    turnMotor =
        new MotionMagicTalonFX(mI.turnMotorID, canbus, turnGains, mI.turnRatio, maxTurnVelo, maxTurnAccel);
    turnMotor.setInverted(mI.turnInverted); // All turn modules were inverted
    applyCurrentLimit(turnMotor, currentLimit);

    driveMotor = new VelocityTalonFX(mI.driveMotorID, canbus, driveGains, mI.driveRatio);
    driveMotor.setInverted(mI.driveInverted);
    applyCurrentLimit(driveMotor, currentLimit);

    MotorOutputConfigs configs = new MotorOutputConfigs();
    driveMotor.getConfigurator().refresh(configs);
    configs.NeutralMode = NeutralModeValue.Brake;
    driveMotor.getConfigurator().apply(configs);

  }

  @Override
  public void applyCurrentLimit(TalonFX motor, CurrentLimitsConfigs limit) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    motor.getConfigurator().refresh(config);
    config.CurrentLimits = limit;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    inputs.turnEncoderPos = turnEncoder.getAbsolutePosition().getValue() * 360;

    inputs.turnMotorAngle = turnMotor.getEncoderPosition();
    inputs.turnMotorVelocity = turnMotor.getEncoderVelocity();
    inputs.turnMotorTarget = turnMotor.getTarget();

    inputs.driveMotorPosition = driveMotor.getEncoderPosition();
    inputs.driveMotorVelocity = driveMotor.getEncoderVelocity();
  }

  @Override
  public Command calculateDriveKV(double kS, Trigger increment, Trigger invert, BooleanSupplier interrupt, boolean telemetry) {
    return driveMotor.calculateKV(kS, 0.05, increment, interrupt);
  }

  @Override
  public Command calculateTurnKV(double kS, Trigger increment, BooleanSupplier interrupt) {
    return turnMotor.calculateKV(kS, 0.05, increment, interrupt);
  }

  @Override
  public void resetTurnMotorPosition(double pos) {
    turnMotor.resetPosition(pos);
  }

@Override
public void setDriveMotorTarget(double target) {
    driveMotor.setTarget(target);
}

@Override
public void setTurnMotorTarget(double target) {
    turnMotor.setTarget(target);
}

  

  
}
