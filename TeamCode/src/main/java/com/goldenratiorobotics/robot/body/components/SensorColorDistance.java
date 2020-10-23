package com.goldenratiorobotics.robot.body.components;

import android.graphics.Color;
import android.service.autofill.SaveCallback;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;
import java.util.Map;

public class SensorColorDistance {

    private static Map<String, SensorColorDistance> map = new HashMap<String, SensorColorDistance>();
    private String myname = null;

    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;

    private final double SCALE_FACTOR = 255;

    private SensorColorDistance(HardwareMap hardwareMap, String name) {
        sensorColor = hardwareMap.get(ColorSensor.class, name);
        sensorDistance = hardwareMap.get(DistanceSensor.class, name);

        myname = name;
    }

    public static SensorColorDistance getInstance(HardwareMap hardwareMap, String name) {
        if(map.get(name)==null)
        {
            map.put(name, new SensorColorDistance(hardwareMap, name));
        }

        return map.get(name);
    }

    public int[] readRGB() {
        int[] rgb = new int[] {(int) (sensorColor.red() * SCALE_FACTOR), (int) (sensorColor.green() * SCALE_FACTOR), (int) (sensorColor.blue() * SCALE_FACTOR)};
        return rgb;
    }

    public float[] readHSV() {
        int[] rgb = readRGB();
        float[] hsv = new float[3];
        Color.RGBToHSV(rgb[0], rgb[1], rgb[2], hsv);
        return hsv;
    }

    public double readDistance(DistanceUnit unit) {
        return sensorDistance.getDistance(unit);
    }
}
