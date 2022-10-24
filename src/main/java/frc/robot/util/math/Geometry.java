package frc.robot.util.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.*;
import static java.lang.Math.*;

public class Geometry {

    public static Rotation2d angleToBetweenPoints(Translation2d originPoint, Translation2d targetPoint) {
        return new Rotation2d(Math.atan2(targetPoint.getY()- originPoint.getY(), targetPoint.getX())- originPoint.getX());
    }

    /**
     * Gets the estimated pose of the target currently being tracked by the limelight based on the gyro and turret angle
     * @return
     */
    public static Pose2d getCurrentTargetPose(Rotation2d gyroAngle, Rotation2d turretAngle, Rotation2d limelightXOffset) {
        double distanceFromCenterToTarget = Units.inchesToMeters(26.687500); //Distance from field center to the tape

        Rotation2d angle = gyroAngle.plus(turretAngle).plus(limelightXOffset).minus(Rotation2d.fromDegrees(180));

        Translation2d translation = moveByAngle(angle, distanceFromCenterToTarget);

        Pose2d target = new Pose2d(GOAL_POS.plus(translation), new Rotation2d());

        return target;
    }

    private static Translation2d moveByAngle(Rotation2d angle, double distance) {
        return new Translation2d(angle.getCos()*distance, angle.getSin()*distance);
    }

    /**
     * Return an orientation for the turret to face towards a target point
     * @param currentPose the current pose of the robot
     * @param targetPoint the point the turret should be targeting
     * @return the angle the turret should be facing to point towards the chosen point
     */
    public static Rotation2d getTurretTarget(Pose2d currentPose, Translation2d targetPoint) {
        //Calculate turret angle

        double xOffset = targetPoint.getX() - currentPose.getX();
        double yOffset = targetPoint.getY() - currentPose.getY();

        //Angle to the goal point (regardless of robot orientation)
        Rotation2d angleToGoal = new Rotation2d(atan2(yOffset, xOffset));

        Rotation2d relAngle = angleToGoal.minus(currentPose.getRotation());

        return relAngle;
    }

    public static Translation2d getEjectionTargetPoint(Pose2d currentPose) {
        return Turret.INVALID_EJECTION_REGION.pointInBoundingRange(currentPose.getTranslation()) ? Turret.OPP_HANGAR_POINT : Turret.OUR_HANGAR_POINT;
    }
}
