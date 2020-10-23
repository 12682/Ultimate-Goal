package com.goldenratiorobotics.robot.body.components;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import android.graphics.Color;

import java.util.HashMap;
import java.util.Map;

public class SensorColor {
    private float[] hsv;
    private int[]   rgb;

    //region simple singleton
    private static Map<String, SensorColor> map = new HashMap<String, SensorColor>();
    private String myname = null;

    private ColorSensor sensor = null;

    private SensorColor(HardwareMap hardwareMap, String name) {
        sensor  = hardwareMap.get(LynxI2cColorRangeSensor.class, name);
        myname = name;
    }

    public static SensorColor getInstance(HardwareMap hardwareMap, String name) {
        if(map.get(name)==null)
        {
            map.put(name, new SensorColor(hardwareMap, name));
        }

        return map.get(name);
    }
    //endregion

    public int[] readRGB() {
        int red   = sensor.red();
        int green = sensor.green();
        int blue  = sensor.blue();

        rgb    = new int[3];
        rgb[0] = red;
        rgb[1] = green;
        rgb[2] = blue;

        return rgb;
    }

    public float[] readHSV() {
        int red   = sensor.red();
        int green = sensor.green();
        int blue  = sensor.blue();

        hsv = new float[3];
        Color.RGBToHSV(red,green,blue, hsv);

        return hsv;
    }
}
