package frc.robot.ShamLib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {


    private NetworkTable table;

    Double[] defaultPose = new Double[]{0.,0.,0.,0.,0.,0.};

    /**
     * Creates a limelight object
     * @param tableID the id of the network table where the limelight will post its data
     */
    public Limelight(String tableID) {
            this.table = getLimeLightTable(tableID);
        }

    /**
     * Creates a limelight object with the default network table name - "limelight"
     */
    public Limelight() {
        this.table = getLimeLightTable("limelight");
    }

    /**
     * @return The limelight's data table (used to access values from the pipeline)
     */
    private NetworkTable getLimeLightTable(String tableID) {
        return NetworkTableInstance.getDefault().getTable(tableID);
    }

    /**
     * @return true if the limelight has identified a valid target
     */
    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public Rotation2d getXOffset() {
        return Rotation2d.fromDegrees(-table.getEntry("tx").getDouble(0));
    }

    public Rotation2d getYOffset() {
        return Rotation2d.fromDegrees(table.getEntry("ty").getDouble(0));

    }

    /**
     * Turns on the limelight
     */
    public void setOn() {
        table.getEntry("ledMode").setNumber(3);
    }

    /**
     * Turns off the limelight
     */
    public void setOff() {
        table.getEntry("ledMode").setNumber(1);
    }

    /**
     * @return the latency of the limelight (in ms)
     */
    public double getLatency() {
        return table.getEntry("tl").getDouble(0);
    }

    public Pose3d getPose3d() {

        Double[] botPose = table.getEntry("botpose").getDoubleArray(defaultPose);

        Pose3d pose = new Pose3d(botPose[0], botPose[1], botPose[2], new Rotation3d(Math.toRadians(botPose[3]), Math.toRadians(botPose[4]), Math.toRadians(botPose[5])));

        return pose;
    }

}
