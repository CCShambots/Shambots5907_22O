package frc.robot.util.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Geometry {

    public static Rotation2d angleToBetweenPoints(Translation2d originPoint, Translation2d targetPoint) {
        return new Rotation2d(Math.atan2(targetPoint.getY()- originPoint.getY(), targetPoint.getX())- originPoint.getX());
    }

}
