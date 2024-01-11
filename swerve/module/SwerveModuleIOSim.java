package frc.robot.ShamLib.swerve.module;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.talonfx.sim.PhysicsSim;

public class SwerveModuleIOSim extends SwerveModuleIOReal {

  public SwerveModuleIOSim(
      String canbus,
      ModuleInfo mI,
      PIDSVGains driveGains,
      PIDSVGains turnGains,
      double maxTurnVelo,
      double maxTurnAccel,
      CurrentLimitsConfigs currentLimit) {
    super(canbus, mI, driveGains, turnGains, maxTurnVelo, maxTurnAccel, currentLimit);

    PhysicsSim.getInstance().addTalonFX(turnMotor, 0.0001);
    PhysicsSim.getInstance().addTalonFX(driveMotor, 0.005);
  }

  // Make sure current limits don't apply because it's sim
  @Override
  public void applyCurrentLimit(TalonFX motor, CurrentLimitsConfigs limit) {}

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    inputs.turnEncoderPos = Math.IEEEremainder(turnMotor.getEncoderPosition(), 360);

    inputs.turnMotorAngle = turnMotor.getEncoderPosition();
    inputs.turnMotorVelocity = turnMotor.getEncoderVelocity();
    inputs.turnMotorTarget = turnMotor.getTarget();

    inputs.driveMotorPosition = driveMotor.getEncoderPosition();
    inputs.driveMotorVelocity = driveMotor.getEncoderVelocity();
    inputs.driveMotorTarget = driveMotor.getTarget();
  }
}
