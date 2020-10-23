package com.goldenratiorobotics.robot.body.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Point;

public class OdometryUnit {
    private DcMotor verticalLeft, verticalRight, horizontal;

    private final double COUNTS_PER_INCH = 307.699557;

    private OdometryGlobalCoordinatePosition globalPositionUpdate;
    private Thread positionThread;

    public OdometryUnit(HardwareMap hardwareMap, String leftPodName, String rightPodName, String horizontalPodName) {
        verticalLeft = hardwareMap.dcMotor.get(leftPodName);
        verticalRight = hardwareMap.dcMotor.get(rightPodName);
        horizontal = hardwareMap.dcMotor.get(horizontalPodName);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Start the thread (should be run after the waitForStart() but before the while loop)
     */
    public void start() {
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        // Maybe don't reverse it to have the right cartesian plane and orientation
//        globalPositionUpdate.reverseNormalEncoder();
    }

    /**
     * Stop the thread (should be run after the while loop)
     */
    public void stop() {
        globalPositionUpdate.stop();
    }

    /**
     * Get the Vertical Left position
     * @return vertical left position
     */
    public int returnVL() {
        return verticalLeft.getCurrentPosition();
    }

    /**
     * Get the Vertical Right position
     * @return vertical right position
     */
    public int returnVR() {
        return verticalRight.getCurrentPosition();
    }

    /**
     * Get the Horizontal position
     * @return horizontal position
     */
    public int returnH() {
        return horizontal.getCurrentPosition();
    }

    /**
     * Returns the X coordinate in inches
     * @return X coordinate inches
     */
    public double returnXIN() {
        return globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH;
    }

    /**
     * Returns the x coordinate in a given unit of distance
     * @param distanceUnit units of distance
     * @return x coordinate
     */
    public double returnXUnits(DistanceUnit distanceUnit) {
        return distanceUnit.fromInches(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
    }

    /**
     * Returns the X coordinate in the given unit of distance
     * @param distanceUnit unit of distance
     * @return X coordinate in given units
     */
    public double returnXDistance(DistanceUnit distanceUnit) {
        return distanceUnit.fromInches(returnXIN());
    }

    /**
     * Returns the X coordinate in centimeters
     * @return X coordinate centimeters
     */
    public double returnXCM() {
        return returnXIN() * 2.54;
    }

    /**
     * Returns the Y coordinate in inches
     * @return Y coordinate inches
     */
    public double returnYIN() {
        return globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH;
    }

    /**
     * Returns the Y coordinate in the given unit of distance
     * @param distanceUnit unit of distance
     * @return Y coordinate in given units
     */
    public double returnYDistance(DistanceUnit distanceUnit) {
        return distanceUnit.fromInches(returnYIN());
    }

    /**
     * Returns the X coordinate in given units of distance
     * @param distanceUnit unit of distance
     * @return X coordinates in given units
     */
    public double returnYUnits(DistanceUnit distanceUnit) {
        return distanceUnit.fromInches(globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
    }

    /**
     * Return the Y coordinate in centimeters
     * @return Y coordinate centimeters
     */
    public double returnYCM() {
        return returnYIN() * 2.54;
    }

    /**
     * Get the orientation in degrees of the robot (globalPosition has a clockwise system, so we invert it)
     * @return Orientation of the robot
     */
    public double returnOrientation() {
        return globalPositionUpdate.returnOrientation();
    }

    /**
     * Find the point (as a double array) of the robot in given distance units
     * @param distanceUnit units of distance to use
     * @return point of the robot in units of distance
     */
    public double[] returnXY(DistanceUnit distanceUnit) {
        return new double[] {returnXDistance(distanceUnit), returnYDistance(distanceUnit)};
    }

    /**
     * Find the current position of the robot in the coordinate plane without units of distance
     * @return point of the robot's position
     */
    public Point returnPoint() {
        return new Point(globalPositionUpdate.returnXCoordinate(), globalPositionUpdate.returnYCoordinate());
    }

    /**
     * Find the current position of the robot in the coordinate plane with the given units of distance
     * @param distanceUnit Units of distance to use
     * @return Point of the robot's position in units of distance
     */
    public Point returnPointUnits(DistanceUnit distanceUnit) {
        return new Point(distanceUnit.fromInches(returnPoint().x / COUNTS_PER_INCH), distanceUnit.fromInches(returnPoint().y / COUNTS_PER_INCH));
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    public double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    public double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Check if the thread is still running
     * @return alive state of the thread
     */
    public boolean isAlive() {
        return positionThread.isAlive();
    }
}
