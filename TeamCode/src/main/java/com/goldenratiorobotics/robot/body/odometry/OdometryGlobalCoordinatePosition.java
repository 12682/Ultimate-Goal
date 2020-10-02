package com.goldenratiorobotics.robot.body.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;


// Based off of code from Wizards.exe (Sarthak)
public class OdometryGlobalCoordinatePosition implements Runnable {
    private DcMotor verticalLeft;
    private DcMotor verticalRight;
    private DcMotor horizontal;

    private boolean isRunning = true;

    // Position variables
    double verticalRightPosition = 0;
    double verticalLeftPosition  = 0;
    double horizontalPosition    = 0;
    double changeInOrientation   = 0;

    private double globalXCoordinate  = 0;
    private double globalYCoordinate  = 0;
    private double orientationRadians = 0;

    private double previousVerticalRightPosition = 0;
    private double previousVerticalLeftPosition  = 0;
    private double previousHorizontalPosition    = 0;

    // Algorithm constants
    private double encoderWheelDistance;
    private double horizontalTickPerDegreeOffset;

    // Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    // Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    private int verticalLeftPositionMultiplier  = 1;
    private int verticalRightPositionMultiplier = 1;
    private int horizontalPositionMultiplier    = 1;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param verticalLeft left pod, facing the vertical direction
     * @param verticalRight right pod, facing the vertical direction
     * @param horizontal horizontal pod, perpendicular to the other two pods
     * @param threadSleepDelay delay in milliseconds (50-75 ms suggested)
     */
    public OdometryGlobalCoordinatePosition(DcMotor verticalLeft, DcMotor verticalRight, DcMotor horizontal, double COUNTS_PER_INCH, int threadSleepDelay) {
        this.verticalLeft = verticalLeft;
        this.verticalRight = verticalRight;
        this.horizontal = horizontal;
        sleepTime = threadSleepDelay;

        encoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    }

    /**
     * Updates the global (x, y, theta) coordinate positions of the robot
     */
    private void globalCoordinatePositionUpdate() {
        // Get current positions
        verticalLeftPosition = (verticalLeft.getCurrentPosition() * verticalLeftPositionMultiplier);
        verticalRightPosition = (verticalRight.getCurrentPosition() * verticalRightPositionMultiplier);

        double leftChange = verticalLeftPosition - previousVerticalLeftPosition;
        double rightChange = verticalRightPosition - previousVerticalRightPosition;

        // Calculate angle
        changeInOrientation = (leftChange - rightChange) / (encoderWheelDistance);
        orientationRadians = (orientationRadians + changeInOrientation);

        // Get the components of the motion
        horizontalPosition = (horizontal.getCurrentPosition() * horizontalPositionMultiplier);
        double rawHorizontalChange = horizontalPosition - previousHorizontalPosition;
        double horizontalChange    = rawHorizontalChange - (changeInOrientation * horizontalTickPerDegreeOffset);

        double p = ((rightChange - leftChange) / 2);
        double n = horizontalChange;

        // Calculate and udpate the position values
        globalXCoordinate = globalXCoordinate + (p * Math.sin(orientationRadians)) + (n * Math.cos(orientationRadians));
        globalYCoordinate = globalYCoordinate + (p * Math.cos(orientationRadians)) + (n * Math.sin(orientationRadians));

        previousVerticalLeftPosition  = verticalLeftPosition;
        previousVerticalRightPosition = verticalRightPosition;
        previousHorizontalPosition    = horizontalPosition;
    }

    /**
     * Returns the global x coordinate
     * @return global x coordinate
     */
    public double getGlobalXCoordinate() {
        return globalXCoordinate;
    }

    /**
     * Returns the global y coordinate
     * @return global y coordinate
     */
    public double getGlobalYCoordinate() {
        return globalYCoordinate;
    }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double getOrientation() {
        return Math.toDegrees(orientationRadians) % 360;
    }

    /**
     * Stops the position update thread
     */
    public void stop() {
        isRunning = false;
    }

    public void reverseLeftEncoder() {
        verticalLeftPositionMultiplier *= -1;
    }

    public void reverseRightEncoder() {
        verticalRightPositionMultiplier *= -1;
    }

    public void reverseHorizontalEncoder() {
        horizontalPositionMultiplier *= -1;
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
