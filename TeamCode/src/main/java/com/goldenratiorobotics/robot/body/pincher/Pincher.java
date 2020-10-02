package com.goldenratiorobotics.robot.body.pincher;

import com.goldenratiorobotics.robot.body.components.LimitSwitchREV;
import com.goldenratiorobotics.robot.body.components.Motor;
import com.goldenratiorobotics.robot.body.components.ServoNonContinuous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Pincher {
    private Motor              motor = null;
    private ServoNonContinuous servo = null;
    private LimitSwitchREV     pivotLimit0 = null;
    private LimitSwitchREV     pivotLimit1 = null;

    //for digital servos: find the pulse width range
    //for analog servos: 500 to 2500
    private double minPWM = 500;
    private double maxPWM = 2500;

    //math to find the minimum and maximum servo position
    private double servoMin = (minPWM - 500) / 2000;
    private double servoMax = (maxPWM - 500) / 2000;

    public Pincher(HardwareMap hardwareMap) {
        motor = Motor.getInstance(hardwareMap, "pivotMotor");
        servo = ServoNonContinuous.getInstance(hardwareMap, "pincherServo");
        pivotLimit0 = LimitSwitchREV.getInstance(hardwareMap, "pivotLimit0");
        pivotLimit1 = LimitSwitchREV.getInstance(hardwareMap, "pivotLimit1");

        motor.zeroPowerBrake();
        //Should we set it to Brake mode or Float mode?
    }

    public void setBrakeMode() {
        motor.zeroPowerBrake();
    }

    public void setFloatMode() {
        motor.zeroPowerFloat();
    }

    //region Variable parameters
    public void setServoMin(double position) {
        servoMin = position;
    }

    public double getServoMin() {
        return servoMin;
    }

    public void setServoMax(double position) {
        servoMax = position;
    }

    public double getServoMax() {
        return servoMax;
    }
    //endregion

    public void pinch() {
        servo.setPosition(servoMax);
    }

    public boolean isPinched() {
        return getServoPosition() == getServoMax();
    }

    public void release() {
        servo.setPosition(servoMin);
    }

    public boolean isReleased() {
        return getServoPosition() < getServoMax();
    }

    public void togglePincher() {
        if (getServoPosition() > ((servoMin + servoMax) / 2)) {
            release();
        } else {
            pinch();
        }
    }

    public void backPivot(double speed) {
        motor.setSpeed(speed);
        if (getStatesLimits()[0]) {
            motor.setSpeed(0);
        }
    }

    public void backPivotAuto(double speed, double timeOut) {
        double st = System.currentTimeMillis();
        motor.setSpeed(speed);
        while (!getStatesLimits()[0] && (System.currentTimeMillis() - st < timeOut)) {
            //wait
        }
        motor.setSpeed(0);
    }

    public boolean isPivotBack() {
        return pivotLimit0.isPressed(); //Need to test motor direction and if switch is inverted
    }

    public void lowerPivot(double speed) {
        motor.setSpeed(-speed);
        if (getStatesLimits()[1]) {
            motor.setSpeed(0);
        }
    }

    public void lowerPivotAuto(double speed, double timeOut) {
        double st = System.currentTimeMillis();
        motor.setSpeed(-speed);
        while (!getStatesLimits()[1] && (System.currentTimeMillis() - st < timeOut)) {
            //wait
        }
        motor.setSpeed(0);
    }

    public boolean isPivotLowered() {
        return pivotLimit1.isPressed(); //Need to test motor direction
    }

    public void runMotorManual(double speed) {
        motor.setSpeed(speed);
    }

    public void runPivot(double speed) {
        if (pivotLimit0.isPressed()) {
            runMotorManual(Range.clip(speed, -1, 0));
        } else if (pivotLimit1.isPressed()) {
            runMotorManual(Range.clip(speed, 0, 1));
        } else {
            runMotorManual(speed);
        }
    }

    public void pivotToPosition(double speed, int position) {
        motor.runToPosition(speed, position);
    }

    public int getMotorPosition() {
        return motor.getPosition();
    }

    public void moveServoManual(double position) {
        servo.setPosition(Range.clip(position, servoMin, servoMax));
    }

    public double getServoPosition() {
        return servo.getPosition();
    }

    public double[] getValuesLimits() {
        double[] limitValues = {pivotLimit0.getValue(), pivotLimit1.getValue()};
        return limitValues;
    }

    public boolean[] getStatesLimits() {
        boolean[] limitStates = {pivotLimit0.isPressed(), pivotLimit1.isPressed()};
        return limitStates;
    }
}
