package com.goldenratiorobotics.robot.brain.smart;

import org.opencv.core.Point;

import java.util.ArrayList;

public class SmartMath {
    /**
     * Simplify a given angle in degrees to a range of -180 to 180
     * @param angle given angle in degrees
     * @return angle in degrees within -180 to 180
     */
    public static double simplifyAngle(double angle) {
        return (angle % 360) - 180;
    }

    /**
     * Calculate cosine with degrees
     * @param degrees angle in degrees
     * @return cosine value of the angle given in degrees
     */
    public static double cos(double degrees) {
        return Math.cos(Math.toRadians(degrees));
    }

    /**
     * Calculate sine with degrees
     * @param degrees angle in degrees
     * @return sine value of the angle given in degrees
     */
    public static double sin(double degrees) {
        return Math.sin(Math.toRadians(degrees));
    }

    /**
     * Calculate atan2 and return degrees instead of radians
     * @param y y value
     * @param x x value
     * @return angle in degrees
     */
    public static double atan2(double y, double x) {
        return Math.toDegrees(Math.atan2(y, x));
    }

    /**
     * Find the distance between two points
     * @param point1 point 1
     * @param point2 point 2
     * @return Distance between the two points
     */
    public static double distanceFormula(Point point1, Point point2) {
        return Math.sqrt(Math.pow(point2.y - point1.y, 2) + Math.pow(point2.x - point1.x, 2));
    }

    /**
     * Find the distance of a point to a line defined by two points
     * @param linePoint1 One point of the line
     * @param linePoint2 Second point of the line
     * @param point Point outside of the line
     * @return Distance to the line defined by the two points
     */
    public static double normalDistance(Point linePoint1, Point linePoint2, Point point) {
        double lineDistance = distanceFormula(linePoint1, linePoint2);
        return (((linePoint2.y - linePoint1.y) * point.x) - ((linePoint2.x - linePoint1.x) * point.y) + (linePoint2.x * linePoint1.y) - (linePoint2.y * linePoint1.x)) / lineDistance;
    }

    /**
     * For SmartPursuit, find the intersections of a line in a circle
     * @param circleCenter center of the circle
     * @param radius radius of the circle
     * @param linePoint1 first point of the line
     * @param linePoint2 second point of the line
     * @return all points of intersection
     */
    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }

        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);

        double quadraticA = 1.0 + Math.pow(m1, 2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2) * x1);

        double quadraticC = ((Math.pow(m1, 2) * Math.pow(x1, 2))) - (2.0 * y1 * m1 * x1) + Math.pow(y1, 2) - Math.pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try{
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = Math.min(linePoint1.x, linePoint2.x);
            double maxX = Math.max(linePoint1.x, linePoint2.x);

            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1, yRoot1));
            }

            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if (xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new Point(xRoot2, yRoot2));
            }
        } catch(Exception ignored) {

        }
        return allPoints;
    }
}
