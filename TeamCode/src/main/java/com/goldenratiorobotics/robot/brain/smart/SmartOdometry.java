package com.goldenratiorobotics.robot.brain.smart;

import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.odometry.OdometryUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Point;

public class SmartOdometry {
    DriveTrain   driveTrain;
    OdometryUnit odometryUnit;

    // Coefficient of heading correction - so that speed does not exceed -1 or 1
    final double HEADING_COEFFICIENT = .004;
    // Coefficient of normal correction - so that speed does not exceed -1 or 1
    final double NORMAL_COEFFICIENT = .015;

    public SmartOdometry(DriveTrain driveTrain, OdometryUnit odometryUnit) {
        this.driveTrain   = driveTrain;
        this.odometryUnit = odometryUnit;
    }

    public void moveForward(DistanceUnit distanceUnit, double distance, double minSpeed, double maxSpeed, double timeOut) {
        double st    = System.currentTimeMillis();
        double speed = maxSpeed;

        double orientation = odometryUnit.returnOrientation();
        double heading     = orientation - 90;
        Point  startPos    = odometryUnit.returnPointUnits(distanceUnit).clone();
        Point  currentPos  = startPos.clone();
        Point  endPos      = new Point(startPos.x + (distance * SmartMath.cos(heading)), startPos.y + (distance * SmartMath.sin(heading)));

        double distanceTraveled = 0;
        double goalDistance     = Math.sqrt(Math.pow(endPos.x - startPos.x, 2) + Math.pow(endPos.y - startPos.y, 2));

        driveTrain.moveForward(speed);
        while ((distanceTraveled < goalDistance) && (System.currentTimeMillis() - st < timeOut)) {
            currentPos       = odometryUnit.returnPointUnits(distanceUnit).clone();
            distanceTraveled = SmartMath.distanceFormula(currentPos, startPos);

            speed = maxSpeed + ((minSpeed - maxSpeed) * (distanceTraveled / goalDistance));

            driveTrain.moveForward(speed);
        }
        driveTrain.stop();
    }

    public void moveForwardToPoint(DistanceUnit distanceUnit, Point endPos, double minSpeed, double maxSpeed, double timeOut) {
        double st = System.currentTimeMillis();

        double moveX, moveY, rotX;

        Point startPos   = odometryUnit.returnPointUnits(distanceUnit);
        Point currentPos = startPos;

        double orientation = odometryUnit.returnOrientation();
        double heading     = Math.toDegrees(Math.atan2(SmartMath.sin(orientation + 90), SmartMath.cos(orientation + 90)));
        double goalHeading = Math.toDegrees(Math.atan2(endPos.y - currentPos.y, endPos.x - currentPos.x));

        double startToEndDistance = SmartMath.distanceFormula(endPos, startPos);
        double distanceFromEnd    = SmartMath.distanceFormula(endPos, currentPos);
        double normalDistance     = SmartMath.normalDistance(startPos, endPos, currentPos);

        moveX = -normalDistance * NORMAL_COEFFICIENT;

        moveY = maxSpeed - ((maxSpeed - minSpeed) * (distanceFromEnd / startToEndDistance));

        double headingError = heading - goalHeading;
        if (headingError > 180) {
            headingError -= 180;
        } else if (headingError < -180) {
            headingError += 180;
        }
        rotX = -headingError * HEADING_COEFFICIENT;

        driveTrain.moveTrig(moveX, moveY, rotX);
        while ((distanceFromEnd > 0) && (System.currentTimeMillis() - st < timeOut)) {
            currentPos  = odometryUnit.returnPointUnits(distanceUnit);
            orientation = odometryUnit.returnOrientation();

            distanceFromEnd = SmartMath.distanceFormula(endPos, currentPos);

            normalDistance = SmartMath.normalDistance(startPos, endPos, currentPos);
            moveX = -normalDistance * NORMAL_COEFFICIENT;

            moveY = maxSpeed - ((maxSpeed - minSpeed) * (distanceFromEnd / startToEndDistance));

            heading = orientation + 90;

            headingError = heading - goalHeading;
            if (headingError > 180) {
                headingError -= 180;
            } else if (headingError < -180) {
                headingError += 180;
            }
            rotX = -headingError * HEADING_COEFFICIENT;

            driveTrain.moveTrig(moveX, moveY, rotX);
        }
    }

    public void moveBackward(DistanceUnit distanceUnit, double distance, double minSpeed, double maxSpeed, double timeOut) {
        double st    = System.currentTimeMillis();
        double speed = maxSpeed;

        double orientation = odometryUnit.returnOrientation();
        double heading     = orientation + 90;
        Point  startPos    = odometryUnit.returnPointUnits(distanceUnit).clone();
        Point  currentPos  = startPos.clone();
        Point  endPos      = new Point(startPos.x + (distance * SmartMath.cos(heading)), startPos.y + (distance * SmartMath.sin(heading)));

        double distanceTraveled = 0;
        double goalDistance     = SmartMath.distanceFormula(endPos, startPos);

        driveTrain.moveBackward(speed);
        while ((distanceTraveled < goalDistance) && (System.currentTimeMillis() - st < timeOut)) {
            currentPos       = odometryUnit.returnPointUnits(distanceUnit).clone();
            distanceTraveled = SmartMath.distanceFormula(currentPos, startPos);

            speed = maxSpeed + ((minSpeed - maxSpeed) * (distanceTraveled / goalDistance));

            driveTrain.moveBackward(speed);
        }
        driveTrain.stop();
    }

    public void moveLeft(DistanceUnit distanceUnit, double distance, double minSpeed, double maxSpeed, double timeOut) {
        double st    = System.currentTimeMillis();
        double speed = maxSpeed;

        double orientation = odometryUnit.returnOrientation();
        double heading     = orientation + 180;
        Point  startPos    = odometryUnit.returnPointUnits(distanceUnit);
        Point  currentPos  = startPos;
        Point  endPos      = new Point(startPos.x + (distance * SmartMath.cos(heading)), startPos.y + (distance * SmartMath.sin(heading)));

        double distanceTraveled = 0;
        double goalDistance     = SmartMath.distanceFormula(endPos, startPos);

        driveTrain.moveLeft(speed);
        while ((distanceTraveled < goalDistance) && (System.currentTimeMillis() - st < timeOut)) {
            currentPos       = odometryUnit.returnPointUnits(distanceUnit);
            distanceTraveled = SmartMath.distanceFormula(currentPos, startPos);

            speed = maxSpeed + ((minSpeed - maxSpeed) * (distanceTraveled / goalDistance));

            driveTrain.moveLeft(speed);
        }
        driveTrain.stop();
    }

    public void moveRight(DistanceUnit distanceUnit, double distance, double minSpeed, double maxSpeed, double timeOut) {
        double st    = System.currentTimeMillis();
        double speed = maxSpeed;

        double orientation = odometryUnit.returnOrientation();
        double heading     = orientation;
        Point  startPos    = odometryUnit.returnPointUnits(distanceUnit);
        Point  currentPos  = startPos;
        Point  endPos      = new Point(startPos.x + (distance * SmartMath.cos(heading)), startPos.y + (distance * SmartMath.sin(heading)));

        double distanceTraveled = 0;
        double goalDistance     = SmartMath.distanceFormula(endPos, startPos);

        driveTrain.moveRight(speed);
        while ((distanceTraveled < goalDistance) && (System.currentTimeMillis() - st < timeOut)) {
            currentPos       = odometryUnit.returnPointUnits(distanceUnit);
            distanceTraveled = SmartMath.distanceFormula(currentPos, startPos);

            speed = maxSpeed + ((minSpeed - maxSpeed) * (distanceTraveled / goalDistance));

            driveTrain.moveRight(speed);
        }
        driveTrain.stop();
    }

    public void rotateRight(double degrees, double minSpeed, double maxSpeed, double timeOut) {
        double st    = System.currentTimeMillis();
        double speed = maxSpeed;

        double startDeg   = odometryUnit.returnOrientation();
        double currentDeg = startDeg;
        double endDeg     = startDeg + degrees;

        driveTrain.rotateRight(speed);
        while ((currentDeg < endDeg) && (System.currentTimeMillis() - st < timeOut)) {
            currentDeg = odometryUnit.returnOrientation();
            speed      = maxSpeed + ((minSpeed - maxSpeed) * (Math.abs(currentDeg - startDeg) / Math.abs(endDeg - startDeg)));

            driveTrain.rotateRight(speed);
        }
        driveTrain.stop();
    }

    public void rotateRightAdjust(double degrees, double minSpeed, double maxSpeed, double timeOut) {
        double st    = System.currentTimeMillis();
        double speed = maxSpeed;

        double startDeg   = odometryUnit.returnOrientation();
        double currentDeg = startDeg;
        double endDeg     = startDeg - degrees;

        driveTrain.rotateRight(speed);
        while (System.currentTimeMillis() - st < timeOut) {
            currentDeg = odometryUnit.returnOrientation();
            if (currentDeg < endDeg) {
                speed = maxSpeed + ((minSpeed - maxSpeed) * Math.abs(currentDeg - startDeg) / Math.abs(endDeg - startDeg));

                driveTrain.rotateLeft(speed);
            } else if (currentDeg > endDeg) {
                speed = maxSpeed + ((minSpeed - maxSpeed) * Math.abs(currentDeg - endDeg) / Math.abs(endDeg - startDeg));

                driveTrain.rotateRight(speed);
            } else {
                driveTrain.stop();
            }
        }
        driveTrain.stop();
    }

    public void rotateLeft(double degrees, double minSpeed, double maxSpeed, double timeOut) {
        double st    = System.currentTimeMillis();
        double speed = maxSpeed;

        double startDeg   = odometryUnit.returnOrientation();
        double currentDeg = startDeg;
        double endDeg     = startDeg - degrees;

        driveTrain.rotateLeft(speed);
        while ((currentDeg > endDeg) && (System.currentTimeMillis() - st < timeOut)) {
            currentDeg = odometryUnit.returnOrientation();
            speed      = maxSpeed + ((minSpeed - maxSpeed) * (Math.abs(currentDeg - startDeg) / Math.abs(endDeg - startDeg)));

            driveTrain.rotateLeft(speed);
        }
        driveTrain.stop();
    }

    public void rotateLeftAdjust(double degrees, double minSpeed, double maxSpeed, double timeOut) {
        double st    = System.currentTimeMillis();
        double speed = maxSpeed;

        double startDeg   = odometryUnit.returnOrientation();
        double currentDeg = startDeg;
        double endDeg     = startDeg + degrees;

        driveTrain.rotateLeft(speed);
        while (System.currentTimeMillis() - st < timeOut) {
            currentDeg = odometryUnit.returnOrientation();
            if (currentDeg < endDeg) {
                speed = maxSpeed + ((minSpeed - maxSpeed) * Math.abs(currentDeg - startDeg) / Math.abs(endDeg - startDeg));

                driveTrain.rotateLeft(speed);
            } else if (currentDeg > endDeg) {
                speed = maxSpeed + ((minSpeed - maxSpeed) * Math.abs(currentDeg - endDeg) / Math.abs(endDeg - startDeg));

                driveTrain.rotateRight(speed);
            } else {
                driveTrain.stop();
            }
        }
        driveTrain.stop();
    }
}
