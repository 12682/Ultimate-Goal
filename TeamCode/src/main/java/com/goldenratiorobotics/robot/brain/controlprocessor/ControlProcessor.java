package com.goldenratiorobotics.robot.brain.controlprocessor;

import com.qualcomm.robotcore.util.Range;

public class ControlProcessor {
    //expo is determined by (1-b)*x+b*x**3 b is loose the exponential graph

    public static double expo(double stick, double expo) {
        return (1 - expo) * stick + expo * Math.pow(stick, 3);
    }

    public static double power(double stick, double power) {
        double value = Range.clip((Math.pow(Math.abs(stick), power)), -1,1);
        if (stick < 0) {
            value = -value;
        }
        return value;
    }

    public static double squareRoot(double stick) {
        return power(stick, 1.0/2.0);
    }

    public static double cubeRoot(double stick) {
        return power(stick, 1.0/3.0);
    }

    public static double stickToServo(double stick) {
        return((stick + 1) / 2);
    }

    public static double stickToContinuous(double stick) {
        return ((stick + 1) / 2);
    }
}
