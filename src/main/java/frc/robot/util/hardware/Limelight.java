package frc.robot.util.hardware;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight{

    private static Limelight instance;

    /**Limelight subsystem that returns all the information we use for targeting */
    private Limelight() {}

    public static Limelight getInstance() {
        if(instance == null) instance = new Limelight();
        return instance;
    }

    /**
     * @return The limelight's data table (used to access values from the pipeline)
     */
    private NetworkTable getLimeLightTable() {
        return NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * @return true if the limelight has identified a valid target
     */
    public boolean hasTarget() {
        return getLimeLightTable().getEntry("tv").getDouble(0) == 1;
    }

    public Rotation2d getXOffset() {
        return Rotation2d.fromDegrees(-getLimeLightTable().getEntry("tx").getDouble(0));
    }

    public Rotation2d getYOffset() {
        return Rotation2d.fromDegrees(getLimeLightTable().getEntry("ty").getDouble(0));

    }

    /**
     * Turns on the limelight
     */
    public void setOn() {
        getLimeLightTable().getEntry("ledMode").setNumber(3);
    }

    /**
     * Turns off the limelight
     */
    public void setOff() {
        getLimeLightTable().getEntry("ledMode").setNumber(1);
    }

    /**
     * @return the latency of the limelight (in ms)
     */
    public double getLatency() {
        return getLimeLightTable().getEntry("tl").getDouble(0);
    }


    //Apriltag stuff
    public Pose3dSendable getPose() {
        return new Pose3dSendable(
                getEntry("botpose/x"),
                getEntry("botpose/y"),
                getEntry("botpose/z"),
                new Rotation3d(
                    getEntry("botpose/roll"),
                    getEntry("botpose/pitch"),
                    getEntry("botpose/yaw")
                )
        );
    }

    public double getEntry(String key) {
        return getLimeLightTable().getEntry(key).getDouble(0);
    }
}
