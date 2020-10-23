package com.goldenratiorobotics.robot.brain.smart;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.brain.gyro.Gyro;

public class SmartTurn {
    private DriveTrain driveTrain = null;
    private Gyro       gyro       = null;

    private double st = 0;

    public SmartTurn(HardwareMap hardwareMap) {
        driveTrain = new DriveTrain(hardwareMap);
        gyro       = new Gyro(hardwareMap);
    }

    public void turnTime(double min, double max, double start, double goal, int timer) {
        st  = System.currentTimeMillis();
        double angle = start;
        double startD = Math.abs(start - goal);
        double currentD = Math.abs(goal - angle);
        double speed = 0;
        while(System.currentTimeMillis() - st < timer && (currentD>1)) {
            angle = gyro.composeAngleYaw();
            currentD = Math.abs(goal - angle);

            speed = min + (max-min)*( currentD / startD );

            if(angle > goal) {
                speed = -speed;
            }
            if(currentD > 2) {
                driveTrain.rotateLeft(speed);
            } else {
                driveTrain.rotateLeft(0);
            }
        }
        driveTrain.reset();
    }

    public void turnTimeNoStart(int timer) {
        st  = System.currentTimeMillis();

        double min = .15;
        double goal = 0;
        double angle = 0;
        double startD = Math.abs(gyro.composeAngleYaw() - goal);
        double max = Range.clip(.7 * (startD / 90) + .15, .15, .9);
        double currentD = Math.abs(goal - angle);
        double speed = 0;
        while(System.currentTimeMillis() - st < timer && (currentD>1)) {
            angle = gyro.composeAngleYaw();
            currentD = Math.abs(goal - angle);

            speed = min + (max-min)*( currentD / startD );

            if(angle > goal) {
                speed = -speed;
            }
            if(currentD > 2) {
                driveTrain.rotateLeft(speed);
            } else {
                driveTrain.rotateLeft(0);
            }
        }
        driveTrain.reset();
    }

    public void setGyroConstant() {
        gyro.setConstant();
    }

    public void turnTo180(double min, double max, int start, int timer) {
        st  = System.currentTimeMillis();
        double angle = 0;
        double startD =  180 - Math.abs(start);
        double currentD = 180 - Math.abs(angle);
        double speed = 0;
        while(System.currentTimeMillis() - st < timer && (currentD>1)) {
            angle = gyro.composeAngleYaw();
            currentD = 180 - Math.abs(angle);

            speed = min + (max-min)*( currentD / startD );

            //angle is negative
            if(angle < 0) {
                speed = -speed;
            }

            driveTrain.rotateLeft(speed); //rotate left if angle is positive
        }
        driveTrain.reset();
    }
}
