package com.goldenratiorobotics.robot.body.components;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class SensorRange {
    //region simple singleton
    private static Map<String, SensorRange> map = new HashMap<String, SensorRange>();

    private String                       myname = null;
    private ModernRoboticsI2cRangeSensor sensor  = null;

    private SensorRange(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, name);
        myname = name;
    }

    public static SensorRange getInstance(HardwareMap hardwareMap, String name) {
        if(map.get(name)==null)
        {
            map.put(name, new SensorRange(hardwareMap, name));
        }

        return map.get(name);
    }
    //endregion

    public double readSmart() {
        double[] readings = {readDistanceCM(), readDistanceCM(), readDistanceCM()};
        Arrays.sort(readings);
        return readings[1];
    }

    public double readDistanceCM() {
        return sensor.getDistance(DistanceUnit.CM);
    }

    public int readUltrasonicRaw() {
        return sensor.rawUltrasonic();
    }

    public int readOpticalRaw() {
        return sensor.rawOptical();
    }

    public double readOpticalCM() {
        return sensor.cmOptical();
    }
}
