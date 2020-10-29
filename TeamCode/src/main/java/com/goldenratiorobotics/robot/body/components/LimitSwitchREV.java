package com.goldenratiorobotics.robot.body.components;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

public class LimitSwitchREV {
    private boolean inverted = false;

    //region simple singleton
    private static Map<String, LimitSwitchREV> map = new HashMap<String, LimitSwitchREV>();

    private String myname   = null;
    RevTouchSensor limit;

    private LimitSwitchREV(HardwareMap hardwareMap, String name) {
        limit  = hardwareMap.get(RevTouchSensor.class, name);
        myname = name;
    }

    public static LimitSwitchREV getInstance(HardwareMap hardwareMap, String name) {

        if(map.get(name)==null)
        {
            map.put(name, new LimitSwitchREV(hardwareMap, name));
        }

        return map.get(name);
    }
    //endregion

    public boolean isPressed() {
        if(!inverted) {
            return limit.isPressed();
        } else {
            return !limit.isPressed();
        }
    }

    public double getValue() {
        return limit.getValue();
    }

    public void invertSwitch() {
        inverted = !inverted;
    }

    public void resetInvertedSwitch() {
        inverted = false;
    }
}
