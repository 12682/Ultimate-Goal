package com.goldenratiorobotics.robot.brain.smart;

import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.odometry.OdometryUnit;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Point;

import java.util.ArrayList;

public class SmartPursuit {

    private DriveTrain driveTrain;
    private OdometryUnit odometryUnit;

    public SmartPursuit(DriveTrain driveTrain, OdometryUnit odometryUnit) {
        this.driveTrain = driveTrain;
        this.odometryUnit = odometryUnit;
    }

    /**
     * Move robot to a specific point in given distance units at a given speed
     * @param distanceUnit units of distance to be used
     * @param endPos end position of the robot
     * @param speed desired speed for the robot
     */
    public void moveToPoint(DistanceUnit distanceUnit, Point endPos, double speed) {
        Point currentPos = odometryUnit.returnPointUnits(distanceUnit);

        double distanceToTarget = Math.hypot(endPos.x - currentPos.x, endPos.y - currentPos.y);
        double absoluteAngleToTarget = SmartMath.atan2(endPos.y - currentPos.y, endPos.x - currentPos.x);
        double relativeAngleToPoint = SmartMath.simplifyAngle(absoluteAngleToTarget - (odometryUnit.returnOrientation() - 90));

        double relativeXToPoint = SmartMath.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = SmartMath.sin(relativeAngleToPoint) * distanceToTarget;
        double absoluteSum = Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint);

        double moveX = (relativeXToPoint / absoluteSum) * speed;
        double moveY = (relativeYToPoint / absoluteSum) * speed;

        driveTrain.moveTrig(moveX, moveY, 0);
    }


    /**
     * Move robot to a specific point in given distance units at a given speed while also rotating
     * @param distanceUnit units of distance to be used
     * @param endPos end position of the robot
     * @param speed desired speed of the robot's movement
     * @param goalAngle desired angle to end at
     * @param turnSpeed desired speed for turning the robot
     */
    public void moveToPointWithAngle(DistanceUnit distanceUnit, Point endPos, double speed, double goalAngle, double turnSpeed) {
        Point currentPos = odometryUnit.returnPointUnits(distanceUnit);

        double distanceToTarget = Math.hypot(endPos.x - currentPos.x, endPos.y - currentPos.y);
        double absoluteAngleToTarget = SmartMath.atan2(endPos.y - currentPos.y, endPos.x - currentPos.x);
        double relativeAngleToPoint = SmartMath.simplifyAngle(absoluteAngleToTarget - (odometryUnit.returnOrientation() - 90));

        double relativeXToPoint = SmartMath.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = SmartMath.sin(relativeAngleToPoint) * distanceToTarget;
        double absoluteSum = Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint);

        double moveX = (relativeXToPoint / absoluteSum) * speed;
        double moveY = (relativeYToPoint / absoluteSum) * speed;

        double relativeTurnAngle = relativeAngleToPoint - 180 + goalAngle;
        double rotX = Range.clip(relativeTurnAngle / 30, -1, 1) * turnSpeed;

        if (distanceToTarget < distanceUnit.fromCm(10)) {
            rotX = 0;
        }

        driveTrain.moveTrig(moveX, moveY, rotX);
    }

    /**
     * Find the path to follow
     * @param distanceUnit units of distance to use
     * @param pathPoints points of the path
     * @param robotPos position of the robot
     * @param followRadius distance from the point to maintain
     * @return the curve point (path) to follow
     */
    public CurvePoint getFollowPointPath(DistanceUnit distanceUnit, ArrayList<CurvePoint> pathPoints, Point robotPos, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = SmartMath.lineCircleIntersection(robotPos, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 1000000;

            for (Point thisIntersection : intersections) {
                double angle = SmartMath.atan2(thisIntersection.y - odometryUnit.returnYUnits(distanceUnit), thisIntersection.x - odometryUnit.returnXUnits(distanceUnit));
                double deltaAngle = Math.abs(SmartMath.simplifyAngle(angle - odometryUnit.returnOrientation()));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }

        return followMe;
    }

    /**
     * Make the robot in curves instead of straight lines
     * @param distanceUnit units of distance to use
     * @param allPoints points to follow
     * @param followAngle angle to go at
     */
    public void followCurve(DistanceUnit distanceUnit, ArrayList<CurvePoint> allPoints, double followAngle) {
        CurvePoint followMe = getFollowPointPath(distanceUnit, allPoints, odometryUnit.returnPointUnits(distanceUnit), allPoints.get(0).followDistance);

        moveToPointWithAngle(distanceUnit, followMe.toPoint(), followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }
}
