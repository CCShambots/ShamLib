package frc.robot.ShamLib.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.UnaryOperator;

public class DriveCommand extends CommandBase{
    private final SwerveDrive drivetrain;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier turnSupplier;

    private final List<SlewRateLimiter> xLimiters = new ArrayList<>();
    private final List<SlewRateLimiter> yLimiters = new ArrayList<>();
    private final List<SlewRateLimiter> thetaLimiters = new ArrayList<>();

    private final List<Double> maxLinearSpeeds = new ArrayList<>();
    private final List<Double> maxRotationalSpeeds = new ArrayList<>();

    private final double deadband;
    private final UnaryOperator<Double> controllerConversion;

    private final boolean useTurning;

    private int prevSpeedMode;

    public DriveCommand(SwerveDrive drivetrain,
                        DoubleSupplier xSupplier,
                        DoubleSupplier ySupplier,
                        DoubleSupplier turnSupplier,
                        double deadband,
                        UnaryOperator<Double> controllerConversion,
                        boolean useTurning,
                        Subsystem subsystem,
                        SwerveSpeedLimits... speedLimits
    ) {
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.turnSupplier = turnSupplier;

        for(SwerveSpeedLimits l : speedLimits) {
            xLimiters.add(new SlewRateLimiter(l.getMaxAcceleration()));
            yLimiters.add(new SlewRateLimiter(l.getMaxAcceleration()));
            thetaLimiters.add(new SlewRateLimiter(l.getMaxRotationalAcceleration()));

            maxLinearSpeeds.add(l.getMaxSpeed());
            maxRotationalSpeeds.add(l.getMaxRotationalSpeed());
        }

        this.deadband = deadband;
        this.controllerConversion = controllerConversion;

        this.useTurning = useTurning;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        resetSpeedLimiters();

        prevSpeedMode = drivetrain.getSpeedMode();
    }

    @Override
    public void execute() {
        int currentSpeedMode = drivetrain.getSpeedMode();

        if(currentSpeedMode != prevSpeedMode) {
            resetSpeedLimiters();
        }

        double correctedX =  convertRawInput(xSupplier.getAsDouble()) * maxLinearSpeeds.get(currentSpeedMode);
        double correctedY =  convertRawInput(ySupplier.getAsDouble()) * maxLinearSpeeds.get(currentSpeedMode);
        double correctedRot =  convertRawInput(turnSupplier.getAsDouble()) * maxRotationalSpeeds.get(currentSpeedMode);

        ChassisSpeeds speeds;

        if(drivetrain.isFieldRelative()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    correctedX, correctedY, correctedRot,
                    drivetrain.getCurrentAngle()
            );
        } else {
            speeds = new ChassisSpeeds(correctedX, correctedY, correctedRot);
        }

        speeds.vxMetersPerSecond = xLimiters.get(currentSpeedMode).calculate(speeds.vxMetersPerSecond);
        speeds.vyMetersPerSecond = yLimiters.get(currentSpeedMode).calculate(speeds.vyMetersPerSecond);
        speeds.omegaRadiansPerSecond = thetaLimiters.get(currentSpeedMode).calculate(speeds.omegaRadiansPerSecond);

        drivetrain.drive(speeds, useTurning, maxLinearSpeeds.get(currentSpeedMode));

        prevSpeedMode = currentSpeedMode;
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

    private void resetSpeedLimiters() {
        ChassisSpeeds currentSpeeds = drivetrain.getChassisSpeeds();
        xLimiters.forEach((e) -> e.reset(currentSpeeds.vxMetersPerSecond));
        yLimiters.forEach((e) -> e.reset(currentSpeeds.vyMetersPerSecond));
        thetaLimiters.forEach((e) -> e.reset(currentSpeeds.omegaRadiansPerSecond));
    }
}
