package frc.robot.util.hardware;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.Turret.*;
import static java.lang.Math.*;

public class Limelight{

    private static Limelight instance;

    /**Limelight subsystem that returns all the information we use for targetting */
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

    /**
     * @return X offset (or yaw)
     */
    public Rotation2d getXOffset() {
        return Rotation2d.fromDegrees(-getLimeLightTable().getEntry("tx").getDouble(0));
    }

    /**
     * @return Y offset (or pitch)
     */
    public Rotation2d getYOffset() {
        return Rotation2d.fromDegrees(getLimeLightTable().getEntry("ty").getDouble(0));
    }


    /**
     * @return distance from center of the field (-1 if no target is found)
     */
    public double getDistanceFromCenter() {
       if(!hasTarget()) return -1;

       double angleRadians = getYOffset().getRadians() + LIMELIGHT_ANGLE;

       double distanceToTape = (GOAL_HEIGHT-LIMELIGHT_HEIGHT) / tan(angleRadians);

       double distanceToCenter = distanceToTape + GOAL_TO_CENTER_DISTANCE;

       return distanceToCenter;
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
}
