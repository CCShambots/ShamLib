package frc.robot.ShamLib.swerve;

public class SwerveSpeedLimits {
    private final double maxSpeed;
    private final double maxAcceleration;
    private final double maxRotationalSpeed;
    private final double maxRotationalAcceleration;

    /**
     * Represents the speed limits that a swerve drive can follow
     * @param maxSpeed the maximum linear speed (m/s)
     * @param maxAcceleration the maximum linear acceleration (m/s^2)
     * @param maxRotationalSpeed the maximum rotational speed (rad/s)
     * @param maxRotationalAcceleration the maximum rotational acceleration (rad/s^2)
     */
    public SwerveSpeedLimits(double maxSpeed, double maxAcceleration, double maxRotationalSpeed, double maxRotationalAcceleration) {
        this.maxSpeed = maxSpeed;
        this.maxAcceleration = maxAcceleration;
        this.maxRotationalSpeed = maxRotationalSpeed;
        this.maxRotationalAcceleration = maxRotationalAcceleration;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public double getMaxRotationalSpeed() {
        return maxRotationalSpeed;
    }

    public double getMaxRotationalAcceleration() {
        return maxRotationalAcceleration;
    }
}
