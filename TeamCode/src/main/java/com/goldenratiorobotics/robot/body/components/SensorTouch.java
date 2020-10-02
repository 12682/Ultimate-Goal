package com.goldenratiorobotics.robot.body.components;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

public class SensorTouch {
    private int inverter = -1;

    //region simple singleton
    private static Map<String, SensorTouch> map = new HashMap<String, SensorTouch>();

    private String myname   = null;
    DigitalChannel sensor;

    private SensorTouch(HardwareMap hardwareMap, String name) {
        sensor  = hardwareMap.get(DigitalChannel.class, name);
        sensor.setMode(DigitalChannel.Mode.INPUT);
        myname = name;
    }

    public static SensorTouch getInstance(HardwareMap hardwareMap, String name) {

        if(map.get(name)==null)
        {
            map.put(name, new SensorTouch(hardwareMap, name));
        }

        return map.get(name);
    }
    //endregion

    public boolean getState() {
        if(inverter == 1) {
            return sensor.getState();
        } else {
            return !sensor.getState();
        }
    }

    public void invertSwitch() {
        inverter = -1;
    }

    public void resetInvertedSwitch() {
        inverter = 1;
    }
}
