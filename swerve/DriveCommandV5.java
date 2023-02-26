package frc.robot.ShamLib.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.DoubleSupplier;
import java.util.function.UnaryOperator;

public class DriveCommandV5 extends CommandBase{
    private SwerveDriveV5 drivetrain;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier turnSupplier;

    private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

    private double maxLinearSpeed;
    private double maxRotationalSpeed;

    private double deadband;
    private UnaryOperator<Double> controllerConversion;

    private boolean useTurning;

    public DriveCommandV5(SwerveDriveV5 drivetrain,
                          DoubleSupplier xSupplier,
                          DoubleSupplier ySupplier,
                          DoubleSupplier turnSupplier,
                          double maxLinearSpeed,
                          double maxLinearAccel,
                          double maxRotationalSpeed,
                          double maxRotAccel,
                          double deadband,
                          UnaryOperator<Double> controllerConversion,
                          boolean useTurning,
                          Subsystem subsystem
    ) {
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.turnSupplier = turnSupplier;

        this.xLimiter = new SlewRateLimiter(maxLinearAccel);
        this.yLimiter = new SlewRateLimiter(maxLinearAccel);
        this.thetaLimiter = new SlewRateLimiter(maxRotAccel);

        this.maxLinearSpeed = maxLinearSpeed;
        this.maxRotationalSpeed = maxRotationalSpeed;
        this.deadband = deadband;
        this.controllerConversion = controllerConversion;

        this.useTurning = useTurning;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double correctedX =  convertRawInput(xSupplier.getAsDouble()) * maxLinearSpeed;
        double correctedY =  convertRawInput(ySupplier.getAsDouble()) * maxLinearSpeed;
        double correctedRot =  convertRawInput(turnSupplier.getAsDouble()) * maxRotationalSpeed;

        ChassisSpeeds speeds;

        if(drivetrain.isFieldRelative()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    correctedX, correctedY, correctedRot,
                    drivetrain.getCurrentAngle()
            );
        } else {
            speeds = new ChassisSpeeds(correctedX, correctedY, correctedRot);
        }

        speeds.vxMetersPerSecond = xLimiter.calculate(speeds.vxMetersPerSecond);
        speeds.vyMetersPerSecond = yLimiter.calculate(speeds.vyMetersPerSecond);
        speeds.omegaRadiansPerSecond = thetaLimiter.calculate(speeds.omegaRadiansPerSecond);

        drivetrain.drive(speeds, useTurning);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
    
    private double convertRawInput(double rawInput){
      double deadbandInput = deadband(rawInput, deadband);
      return controllerConversion.apply(Double.valueOf(deadbandInput));
    }
    
    private double deadband(double rawInput, double deadband){
      if (Math.abs(rawInput) > deadband) {
        if (rawInput > 0.0) return (rawInput - deadband) / (1.0 - deadband);
        else return (rawInput + deadband) / (1.0 - deadband);
      } else return 0;
    }
}
