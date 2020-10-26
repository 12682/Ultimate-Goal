package com.goldenratiorobotics.robot.body.drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.goldenratiorobotics.robot.body.components.Motor;

public class DriveTrain {
    private Motor rightFront = null;
    private Motor leftFront  = null;
    private Motor rightBack  = null;
    private Motor leftBack   = null;

    public DriveTrain(HardwareMap hardwareMap) {
        leftFront  = Motor.getInstance(hardwareMap, "leftFront");
        rightFront = Motor.getInstance(hardwareMap, "rightFront");
        leftBack   = Motor.getInstance(hardwareMap, "leftBack");
        rightBack  = Motor.getInstance(hardwareMap, "rightBack");

        rightFront.reverseDirection();
        rightBack.reverseDirection();
    }

    public void reset() {
        rightFront.stopReset();
        rightBack.stopReset();
        leftFront.stopReset();
        leftBack.stopReset();
    }

    public boolean driveTrainIsBusy() {
        //if one is not busy return not busy (true)
        if(rightFront.isBusy() && rightBack.isBusy() && leftBack.isBusy() && leftFront.isBusy()) {
            return true;
        } else {
            reset(); //reset if drivetrain is not busy
            return false;
        }
    }

    public void runRightFront(double speed) {
        rightFront.setSpeed(speed);
    }

    public void runRightBack(double speed) {
        rightBack.setSpeed(speed);
    }

    public void runLeftFront(double speed) {
        leftFront.setSpeed(speed);
    }

    public void runLeftBack(double speed) {
        leftBack.setSpeed(speed);
    }

    public void runAll(double speed) {
        runRightFront(speed);
        runRightBack(speed);
        runLeftFront(speed);
        runLeftBack(speed);
    }

    public void stop() {
        runAll(0);
    }

    public void testDriveTrain(boolean a, boolean b, boolean x, boolean y) {
        if(a) { leftBack.setSpeed(.3);   }
        if(b) { rightBack.setSpeed(.3);  }
        if(x) { leftFront.setSpeed(.3);  }
        if(y) { rightFront.setSpeed(.3); }
    }

    //region get positions
    public int getPositionAll() {
        return (rightFront.getPosition() + rightBack.getPosition() + leftFront.getPosition() + leftBack.getPosition())/4;
    }

    public int getPositionRightFront() {
        return rightFront.getPosition();
    }

    public int getPositionLeftFront() {
        return leftFront.getPosition();
    }

    public int getPositionRightBack() {
        return rightBack.getPosition();
    }

    public int getPositionLeftBack() {
        return leftBack.getPosition();
    }
    //endregion

    //Deprecated
    public void moveSimple(double rotX, double moveY, double moveX) {
        //NOT FOR GOBILDA CHASSIS!
        //  ___________Movements____________
        // | Left:   1 -1 | Right:     -1 1 |
        // |         1 -1 |            -1 1 |
        // |______________|_________________|
        // |Forward: 1  1 | Backward: -1 -1 |
        // |         1  1 |           -1 -1 |
        // |______________|_________________|
        //
        //  _________Rotations__________
        // | Left:  1 -1 | Right: -1  1 |
        // |       -1  1 |         1 -1 |
        // |_____________|______________|

        leftFront.setSpeed(moveY - moveX + rotX);
        rightFront.setSpeed(moveY + moveX - rotX);
        leftBack.setSpeed(moveY + moveX + rotX);
        rightBack.setSpeed( moveY - moveX - rotX);
    }

    /**
     * Main method to move the robot in teleop
     * @param moveX speed for the x direction of the robot (left and right)
     * @param moveY speed for the y direction of the robot (forward and back)
     * @param rotX speed for the rotation of the robot
     */
    public void moveTrig(double moveX, double moveY, double rotX) {
        // Speed for the robot's strafing
        double r = Math.hypot(moveX, moveY);
        // Angle to move the robot in
        double robotAngle = Math.atan2(moveY, moveX) - (Math.PI / 4);
        // Speeds for the four motors
        double lf = r * Math.cos(robotAngle) + rotX;
        double rf = r * Math.sin(robotAngle) - rotX;
        double lb = r * Math.sin(robotAngle) + rotX;
        double rb = r * Math.cos(robotAngle) - rotX;

        manualMoveAll(lf, rf, lb, rb);
    }

    // Method to move the robot at a desired direction
    public void moveTheta(double speed, double theta) {
        // Left front speed
        double lf = speed * Math.cos(Math.toRadians(theta) - (Math.PI / 4));
        // Right front speed
        double rf = speed * Math.sin(Math.toRadians(theta) - (Math.PI / 4));
        // Left back speed
        double lb = speed * Math.sin(Math.toRadians(theta) - (Math.PI / 4));
        // Right back speed
        double rb = speed * Math.cos(Math.toRadians(theta) - (Math.PI / 4));

        // Set the motors to the specific speeds
        manualMoveAll(lf, rf, lb, rb);
    }

    //region basic movements
    public void moveRight(double speed) {
        leftFront.setSpeed(speed);
        rightFront.setSpeed( -(speed));
        leftBack.setSpeed( -(speed));
        rightBack.setSpeed(speed);
    }

    public void moveLeft(double speed) {
        moveRight( -(speed));
    }

    public void moveForward(double speed) {
        leftFront.setSpeed(speed);
        rightFront.setSpeed(speed);
        leftBack.setSpeed(speed);
        rightBack.setSpeed(speed);
    }

    public void moveBackward(double speed) {
        moveForward( -(speed));
    }

    public void rotateRight(double speed) {
        leftFront.setSpeed(speed);
        rightFront.setSpeed( -(speed));
        leftBack.setSpeed(speed);
        rightBack.setSpeed( -(speed));
    }

    public void rotateLeft(double speed) {
        rotateRight( -(speed));
    }
    //endregion

    //region encoder movements
    public void moveRightToPosition(double speed, int encoderValue) {
        leftFront.runToPosition(speed, encoderValue);
        rightFront.runToPosition( -(speed), -(encoderValue));
        leftBack.runToPosition( -(speed), -(encoderValue));
        rightBack.runToPosition(speed, encoderValue);
        while(driveTrainIsBusy()) { /* wait */ }
        stop();
    }

    public void moveLeftToPosition(double speed, int encoderValue) {
        moveRightToPosition( -(speed), -(encoderValue));
    }

    public void moveForwardToPosition(double speed, int encoderValue) {
        leftFront.runToPosition(speed, encoderValue);
        rightFront.runToPosition(speed, encoderValue);
        leftBack.runToPosition(speed, encoderValue);
        rightBack.runToPosition(speed, encoderValue);
        while(driveTrainIsBusy()) { /* wait */ }
        stop();
    }

    public void moveBackwardToPosition(double speed, int encoderValue) {
        moveForwardToPosition( -(speed), -(encoderValue));
    }
    //endregion

    //region timed movements (milliseconds)
    public void moveRightTimed(double speed, double time) {
        double start = System.currentTimeMillis();
        leftFront.setSpeed(speed);
        rightFront.setSpeed( -(speed));
        leftBack.setSpeed( -(speed));
        rightBack.setSpeed(speed);
        while( System.currentTimeMillis() - start < time) { /* wait */ }
        stop();
    }

    public void moveLeftTimed(double speed, double time) {
        moveRightTimed( -(speed), time);
    }

    public void moveForwardTimed(double speed, double time) {
        double start = System.currentTimeMillis();
        leftFront.setSpeed(speed);
        rightFront.setSpeed(speed);
        leftBack.setSpeed(speed);
        rightBack.setSpeed(speed);
        while( System.currentTimeMillis() - start < time) { /* wait */ }
        stop();
    }

    public void moveBackwardTimed(double speed, double time) {
        moveForwardTimed( -(speed), time);
    }
    //endregion

    public void moveRightModifier(double speed, double modifier) {
        //modifier default is 1 if robot is perfectly centered
        leftFront.setSpeed(speed);
        rightFront.setSpeed( -(speed));
        leftBack.setSpeed( -(speed) * modifier);
        rightBack.setSpeed(speed * modifier);
    }

    public void moveLeftModifier(double speed, double modifier) {
        moveRightModifier( -(speed), modifier);
    }

    public void moveForwardSmart(int encoder, double min, double max, int time) {
        moveForwardToPosition(min, encoder);
        double st = System.currentTimeMillis();
        double maxD = Math.abs(encoder);
        while(driveTrainIsBusy() && (System.currentTimeMillis()-st<time) )
        {
            moveForward( min + (max-min)*( (maxD-getPositionAll())/maxD )  );
        }
        reset();
    }

    public void manualMoveAll(double speedLF, double speedRF, double speedLB, double speedRB) {
        leftFront.setSpeed(speedLF);
        rightFront.setSpeed(speedRF);
        leftBack.setSpeed(speedLB);
        rightBack.setSpeed(speedRB);
    }

    public double getSpeedLF() {
        return leftFront.getSpeed();
    }

    public double getSpeedRF() {
        return rightFront.getSpeed();
    }

    public double getSpeedLB() {
        return leftBack.getSpeed();
    }

    public double getSpeedRB() {
        return rightBack.getSpeed();
    }

    public void zeroPowerFloat() {
        leftBack.zeroPowerFloat();
        rightBack.zeroPowerFloat();
        leftFront.zeroPowerFloat();
        rightFront.zeroPowerFloat();
    }

    public void zeroPowerBrake() {
        leftFront.zeroPowerBrake();
        rightFront.zeroPowerBrake();
        leftBack.zeroPowerBrake();
        rightBack.zeroPowerBrake();
    }
}
