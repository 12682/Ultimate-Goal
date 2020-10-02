package com.goldenratiorobotics.robot.body.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;
import java.util.Map;

public class ServoContinuous {
    //region simple singleton
    private static Map<String, ServoContinuous> map = new HashMap<String, ServoContinuous>();

    private String myname = null;
    private Servo  servo  = null;

    private ServoContinuous(HardwareMap hardwareMap, String name) {
        servo  = hardwareMap.get(Servo.class, name);
        myname = name;
    }

    public static ServoContinuous getInstance(HardwareMap hardwareMap, String name) {
        if (map.get(name) == null) {
            map.put(name, new ServoContinuous(hardwareMap, name));
        }

        return map.get(name);
    }
    //endregion

    public void reverseDirection() {
        servo.setDirection(Servo.Direction.REVERSE);
    }

    public void setSpeed(double speed) {
        servo.setPosition(Range.clip(speed, 0, 1));
    }

    public void runReverse() {
        setSpeed(0);
    }

    public void stop() {
        setSpeed(.5);
    }

    public void runForward() {
        setSpeed(1);
    }

    public double getSpeed() {
        return servo.getPosition();
    }
}
