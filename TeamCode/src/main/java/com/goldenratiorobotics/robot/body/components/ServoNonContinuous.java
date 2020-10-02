package com.goldenratiorobotics.robot.body.components;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;
import java.util.Map;

public class ServoNonContinuous {
    //region simple singleton
    private static Map<String, ServoNonContinuous> map = new HashMap<String, ServoNonContinuous>();

    private String myname = null;
    private Servo  servo  = null;

    private ServoNonContinuous(HardwareMap hardwareMap, String name) {
        servo  = hardwareMap.get(Servo.class, name);
        myname = name;
    }

    public static ServoNonContinuous getInstance(HardwareMap hardwareMap, String name) {
        if(map.get(name) == null) {
            map.put(name, new ServoNonContinuous(hardwareMap, name));
        }

        return map.get(name);
    }
    //endregion

    public double getPosition() {
        return servo.getPosition();
    }

    //FOR ANALOG RANGE IS 0-1
    //FOR DIGITAL IT IS (PWM - 500) / 2000
    public void setPosition(double position) {
        servo.setPosition(Range.clip(position, 0, 1));
    }
}
