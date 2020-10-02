package com.goldenratiorobotics.robot.body.components;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;
import java.util.Map;

public class SensorRangeREV {
    //region simple singleton
    private static Map<String, SensorRangeREV> map = new HashMap<String, SensorRangeREV>();
    private String myname = null;

    private DistanceSensor sensor = null;

    private SensorRangeREV(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(DistanceSensor.class, name);
        myname = name;
    }

    public static SensorRangeREV getInstance(HardwareMap hardwareMap, String name) {
        if(map.get(name)==null) {
            map.put(name, new SensorRangeREV(hardwareMap, name));
        }

        return map.get(name);
    }
    //endregion

    public double readSmart(double numReRead) {
        double reading = readCM();
        if( reading > numReRead ) {
            reading = readCM();
        }
        return reading;
    }

    public double readCM() {
        return sensor.getDistance(DistanceUnit.CM);
    }

    public double readMM() {
        return sensor.getDistance(DistanceUnit.MM);
    }

    public double readIN() {
        return sensor.getDistance(DistanceUnit.INCH);
    }
}
