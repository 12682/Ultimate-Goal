package com.goldenratiorobotics.robot.brain.smart;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.goldenratiorobotics.robot.body.components.SensorRangeREV;
import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;

public class SmartRange {
    private DriveTrain     driveTrain = null;
    private SensorRangeREV leftRange  = null;
    private SensorRangeREV frontRange = null;
    private SensorRangeREV rightRange = null;
    private SensorRangeREV backRange  = null;

    private double st = 0;

    public SmartRange(HardwareMap hardwareMap, boolean left, boolean front, boolean right, boolean back) {
        driveTrain = new DriveTrain(hardwareMap);
        if (left) {
            leftRange = SensorRangeREV.getInstance(hardwareMap, "leftRange");
        }
        if (front) {
            frontRange = SensorRangeREV.getInstance(hardwareMap, "frontRange");
        }
        if (right) {
            rightRange = SensorRangeREV.getInstance(hardwareMap, "rightRange");
        }
        if (back) {
            backRange = SensorRangeREV.getInstance(hardwareMap, "backRange");
        }
    }

    public void moveForwardWithFrontSensor(double min, double max, int start, int end, int timer) {
        st  = System.currentTimeMillis();
        double current = start;
        double startD = Math.abs(start - end);
        double currentD = Math.abs(current - end);
        double speed = 0;
        while(System.currentTimeMillis() - st < timer && ( current > end)) {
            current  = frontRange.readSmart(200);
            currentD = Math.abs(current - end);

            speed = min + (max-min)*( currentD / startD );

            driveTrain.moveForward(speed);
        }
        driveTrain.reset();
    }

    public void moveForwardWithLeftSensor(double min, double max, int start, int end, int timer) {
        st  = System.currentTimeMillis();
        double current = start;
        double startD = Math.abs(start - end);
        double currentD = Math.abs(current - end);
        double speed = 0;
        while(System.currentTimeMillis() - st < timer && ( current > end)) {
            current = leftRange.readSmart(200);
            currentD = Math.abs(current - end);

            speed = min + (max-min)*( currentD / startD );

            driveTrain.moveForward(speed);
        }
        driveTrain.reset();
    }

    public void moveForwardWithRightSensor(double min, double max, int start, int end, int timer) {
        st  = System.currentTimeMillis();
        double current = start;
        double startD = Math.abs(start - end);
        double currentD = Math.abs(current - end);
        double speed = 0;
        while(System.currentTimeMillis() - st < timer && ( current > end)) {
            current = rightRange.readSmart(200);
            currentD = Math.abs(current - end);

            speed = min + (max-min)*( currentD / startD );

            driveTrain.moveForward(speed);
        }
        driveTrain.reset();
    }

    public void moveForwardWithBackSensor(double min, double max, int start, int end, int timer) {
        st  = System.currentTimeMillis();
        double current = start;
        double startD = Math.abs(start - end);
        double currentD = Math.abs(current - end);
        double speed = 0;
        while(System.currentTimeMillis() - st < timer && ( current > end)) {
            current = backRange.readSmart(200);
            currentD = Math.abs(current - end);

            speed = min + (max-min)*( currentD / startD );

            driveTrain.moveForward(speed);
        }
        driveTrain.reset();
    }

    public void moveRightWithRightSensor(double min, double max, int start, int end, int timer) {
        st  = System.currentTimeMillis();
        double current = start;
        double startD = Math.abs(start - end);
        double currentD = Math.abs(current - end);
        double speed = 0;
        while(System.currentTimeMillis() - st < timer && ( current > end)) {
            current = rightRange.readSmart(200);
            currentD = Math.abs(current - end);

            speed = min + (max-min)*( currentD / startD );

            driveTrain.moveRight(speed);
        }
        driveTrain.reset();
    }

    public void moveRightWithLeftSensor(double min, double max, int start, int end, int timer) {
        st  = System.currentTimeMillis();
        double current = start;
        double startD = Math.abs(start - end);
        double currentD = Math.abs(current - end);
        double speed = 0;
        while(System.currentTimeMillis() - st < timer && ( current > end)) {
            current = leftRange.readSmart(200);
            currentD = Math.abs(current - end);

            speed = min + (max-min)*( currentD / startD );

            driveTrain.moveRight(speed);
        }
        driveTrain.reset();
    }

    public void moveLeftWithLeftSensor(double min, double max, int start, int end, int timer) {
        st  = System.currentTimeMillis();
        double current = start;
        double startD = Math.abs(start - end);
        double currentD = Math.abs(current - end);
        double speed = 0;
        while(System.currentTimeMillis() - st < timer && ( current > end)) {
            current = leftRange.readSmart(200);
            currentD = Math.abs(current - end);

            speed = min + (max-min)*( currentD / startD );

            driveTrain.moveLeft(speed);
        }
        driveTrain.reset();
    }

    public void moveLeftWithRightSensor(double min, double max, int start, int end, int timer) {
        st  = System.currentTimeMillis();
        double current = start;
        double startD = Math.abs(start - end);
        double currentD = Math.abs(current - end);
        double speed = 0;
        while(System.currentTimeMillis() - st < timer && ( current > end)) {
            current = rightRange.readSmart(200);
            currentD = Math.abs(current - end);

            speed = min + (max-min)*( currentD / startD );

            driveTrain.moveLeft(speed);
        }
        driveTrain.reset();
    }

    public void moveBackwardWithRightSensor(double min, double max, int start, int end, int timer) {
        st  = System.currentTimeMillis();
        double current = start;
        double startD = Math.abs(end - start); // i.e. startD = abs(100-12) = 88
        double currentD = Math.abs(end-current);
        double speed = 0;
        while(System.currentTimeMillis() - st < timer && ( current < end)) {
            current = rightRange.readSmart(200);
            currentD = Math.abs(end-current); // i.e. currentD = abs(100-50) = 50

            speed = min + (max-min)*( currentD / startD );

            driveTrain.moveBackward(speed);
        }
        driveTrain.reset();
    }

    public void moveBackwardWithLeftSensor(double min, double max, int start, int end, int timer) {
        st  = System.currentTimeMillis();
        double current = start;
        double startD = Math.abs(end - start); // i.e. startD = abs(100-12) = 88
        double currentD = Math.abs(end-current);
        double speed = 0;
        while(System.currentTimeMillis() - st < timer && ( current < end)) {
            current = leftRange.readSmart(200);
            currentD = Math.abs(end-current); // i.e. currentD = abs(100-50) = 50

            speed = min + (max-min)*( currentD / startD );

            driveTrain.moveBackward(speed);
        }
        driveTrain.reset();
    }
}
