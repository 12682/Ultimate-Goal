package com.goldenratiorobotics.robot.body.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OdometryUnit {
    private DcMotor verticalLeft;
    private DcMotor verticalRight;
    private DcMotor horizontal;

    private final double COUNTS_PER_INCH = 307.699557;

    private OdometryGlobalCoordinatePosition globalPositionUpdate;
    private Thread positionThread;

    public OdometryUnit(HardwareMap hardwareMap, String vlName, String vrName, String hName) {
        verticalLeft  = hardwareMap.dcMotor.get(vlName);
        verticalRight = hardwareMap.dcMotor.get(vrName);
        horizontal    = hardwareMap.dcMotor.get(hName);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Run before the while loop, but after the wait for start
     */
    public void startThread() {
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseHorizontalEncoder();
    }

    /**
     * Get the encoder ticks & angle
     * @return array of x, y, theta (degrees)
     */
    public double[] getXYTheta() {
        return new double[] {globalPositionUpdate.getGlobalXCoordinate(), globalPositionUpdate.getGlobalYCoordinate(), getTheta()};
    }

    /**
     * Get the orientation of the robot
     * @return angle
     */
    public double getTheta() {
        return globalPositionUpdate.getOrientation();
    }

    public double getXIN() {
        return (globalPositionUpdate.getGlobalXCoordinate() / COUNTS_PER_INCH);
    }

    public double getXCM() {
        return getXIN() / 0.394;
    }

    public double getYIN() {
        return (globalPositionUpdate.getGlobalYCoordinate() / COUNTS_PER_INCH);
    }

    public double getYCM() {
        return getYIN() / 0.394;
    }

    public int getVerticalLeftPosition() {
        return verticalLeft.getCurrentPosition();
    }

    public int getVerticalRightPosition() {
        return verticalRight.getCurrentPosition();
    }

    public int getHorizontalPosition() {
        return horizontal.getCurrentPosition();
    }

    public boolean isThreadActive() {
        return positionThread.isAlive();
    }

    /**
     * Run after the while loop
     */
    public void stopThread() {
        globalPositionUpdate.stop();
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
}
