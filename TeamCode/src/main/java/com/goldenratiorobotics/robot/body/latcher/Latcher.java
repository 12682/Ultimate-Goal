package com.goldenratiorobotics.robot.body.latcher;

import com.goldenratiorobotics.robot.body.components.ServoNonContinuous;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Latcher {
    private ServoNonContinuous left;
    private ServoNonContinuous right;

    private double latchPositionLeft    = 0;
    private double latchPositionRight   = 1;
    private double unlatchPositionLeft  = 1;
    private double unlatchPositionRight = 0;

    private double[] neutralPositions = {.5, .5};


    public Latcher(HardwareMap hardwareMap) {
        left  = ServoNonContinuous.getInstance(hardwareMap, "latcherLeft");
        right = ServoNonContinuous.getInstance(hardwareMap, "latcherRight");
    }

    public void manualMoveLatchers(double leftPosition, double rightPosition) {
        left.setPosition(leftPosition);
        right.setPosition(rightPosition);
    }

    public double getLeftPosition() {
        return left.getPosition();
    }

    public double getRightPosition() {
        return right.getPosition();
    }

    public double[] getLatchersPositions() {
        double[] positions = {getLeftPosition(), getRightPosition()};
        return positions;
    }

    public void latch() {
        manualMoveLatchers(latchPositionLeft, latchPositionRight);
    }

    public void unlatch() {
        manualMoveLatchers(unlatchPositionLeft, unlatchPositionRight);
    }

    public void neuter() {
        manualMoveLatchers(neutralPositions[0], neutralPositions[1]);
    }

    public void toggle() {
        if ((getLatchersPositions()[0] < neutralPositions[0]) && (getLatchersPositions()[1] > neutralPositions[1])) {
            unlatch();
        } else {
            latch();
        }
    }
}
