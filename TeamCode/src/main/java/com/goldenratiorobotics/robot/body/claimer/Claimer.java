package com.goldenratiorobotics.robot.body.claimer;

import com.goldenratiorobotics.robot.body.components.ServoNonContinuous;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claimer {
    private ServoNonContinuous servo = null;

    private double claimPosition = 0;
    private double neutralPosition = 1;

    public Claimer(HardwareMap hardwareMap) {
        servo = ServoNonContinuous.getInstance(hardwareMap, "claimer");
    }

    public void moveServoManual(double position) {
        servo.setPosition(position);
    }

    public double getServoPosition() {
        return servo.getPosition();
    }

    public void claim() {
        servo.setPosition(claimPosition);
    }

    public void neuter() {
        servo.setPosition(neutralPosition);
    }

    public void toggle() {
        if (getServoPosition() > .5) {
            neuter();
        } else {
            claim();
        }
    }

    public void setNeutralPosition(double position) {
        neutralPosition = position;
    }

    public double getNeutralPosition() {
        return neutralPosition;
    }

    public void setClaimPosition(double position) {
        claimPosition = position;
    }

    public double getClaimPosition() {
        return claimPosition;
    }
}
