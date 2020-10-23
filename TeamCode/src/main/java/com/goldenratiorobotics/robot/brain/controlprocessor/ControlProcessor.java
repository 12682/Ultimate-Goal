package com.goldenratiorobotics.robot.brain.controlprocessor;

import com.qualcomm.robotcore.util.Range;

public class ControlProcessor {
    //expo is determined by (1-b)*x+b*x**3 b is loose the exponential graph

    public static double expo(double stick, double expo) {
        return (1 - expo) * stick + expo * Math.pow(stick, 3);
    }

    public static double power(double stick, double power) {
        if (stick == 0) {
            return 0;
        } else {
            return (stick / Math.abs(stick)) * Math.pow(Math.abs(stick), power);
        }
    }

    public static double squareRoot(double stick) {
        return power(stick, 1.0/2.0);
    }

    public static double cubeRoot(double stick) {
        return power(stick, 1.0/3.0);
    }

    public static double stickToMinMax(double stick, double min, double max) {
        return ((((max - min) / 2) * stick) + ((max + min) / 2));
    }

    public static double halfStickToMinMax(double stick, double min, double max) {
        if (stick <= 0) {
            return min;
        } else {
            return (((max - min) * stick) + min);
        }
    }

    public static double stickToServo(double stick) {
        return stickToMinMax(stick, 0, 1);
    }
}
