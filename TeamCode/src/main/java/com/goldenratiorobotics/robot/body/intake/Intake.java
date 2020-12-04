package com.goldenratiorobotics.robot.body.intake;

import com.goldenratiorobotics.robot.body.components.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private Motor intakeMotor = null;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = Motor.getInstance(hardwareMap, "intakeMotor");

    }

    public void setSpeed(double speed) {
        intakeMotor.setSpeed(speed);

    }
    public void takeIn() {
        setSpeed(-.8);

    }
    public void takeOut() {
        setSpeed(.75);
    }
    public void stop() {
        setSpeed(0);
    }

    public double getSpeed() {
        return intakeMotor.getSpeed();

    }

}
