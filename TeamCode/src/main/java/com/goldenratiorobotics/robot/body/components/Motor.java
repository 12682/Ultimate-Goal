package com.goldenratiorobotics.robot.body.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;
import java.util.Map;

public class Motor {
    //region simple singleton
    private static Map<String, Motor> map = new HashMap<String, Motor>();

    private String  myname = null;
    private DcMotor motor  = null;

    private Motor(HardwareMap hardwareMap, String name) {
        motor  = hardwareMap.get(DcMotor.class, name);
        myname = name;
    }

    public static Motor getInstance(HardwareMap hardwareMap, String name) {
        if(map.get(name) == null) {
            map.put(name, new Motor(hardwareMap, name));
        }

        return map.get(name);
    }
    //endregion

    public void reverseDirection(){
        motor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void zeroPowerFloat() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void zeroPowerBrake() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setSpeed(double speed) {
        motor.setPower(Range.clip(speed, -1, 1));
    }

    public void stop() {
        setSpeed(0);
    }

    public void reset() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void stopReset() {
        stop();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stop();
    }

    public void runToPositionNoReset(double speed, int encoderValue) {
        stop();
        motor.setTargetPosition(encoderValue);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setSpeed(speed);
    }

    public void runToPosition(double speed, int encoderValue) {
        stopReset();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(encoderValue);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setSpeed(speed);
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    public double getSpeed() {
        return motor.getPower();
    }

    public boolean isBusy() {
        return motor.isBusy();
    }
}
