package frc.robot.util.math;

import edu.wpi.first.math.geometry.Translation2d;

import java.awt.*;
import java.util.List;

public class BoundingRegion {
    private Polygon polygon;

    public BoundingRegion(List<Translation2d> boundingPoints) {
        int[][] array = getPointArrays(boundingPoints);
        polygon = new Polygon(array[0], array[1], boundingPoints.size());
    }

    public BoundingRegion(Translation2d... boundingPoints) {
        this(List.of(boundingPoints));
    }

    /**
     * Multiples the points by 100 to retain up to 2 decimal places of preceision, and returns them in the proper form for the Java Polygon
     * @param points
     * @return
     */
    private int[][] getPointArrays(List<Translation2d> points) {
        int[][] array = new int[2][points.size()];

        for(int i = 0; i<points.size(); i++) {
            array[0][i] = (int) points.get(i).getX() * 100;
            array[1][i] = (int) points.get(i).getY() * 100;
        }

        return array;
    }

    /**
     * Performs the even-odd-rule Algorithm to find out whether a point is in a given polygon.
     * This runs in O(n) where n is the number of edges of the polygon.
     *
     * @param point a translation for the point you want to check.
     * @return whether the point is in the polygon (not on the edge, just turn < into <= and > into >= for that)
     */
    public boolean pointInBoundingRange(Translation2d point) {
        Translation2d multipliedPoint = new Translation2d(point.getX(), point.getY());

        return polygon.contains(multipliedPoint.getX(), multipliedPoint.getY());
    }
}
