package org.firstinspires.ftc.teamcode.tests.components;

import com.goldenratiorobotics.robot.body.components.SensorColor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test Sensor Color", group="D. Tests Components")
//@Disabled
public class TestSensorColor extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private SensorColor sensor = null;

    @Override
    public void runOpMode() {
        sensor = SensorColor.getInstance(hardwareMap, "bottomColor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("HSV", sensor.readHSV()[0] + ", " + sensor.readHSV()[1] + ", " + sensor.readHSV()[2]);
            telemetry.addData("RGB", sensor.readRGB()[0] + ", " + sensor.readRGB()[1] + ", " + sensor.readRGB()[2]);
            telemetry.update();
            //endregion
        }
    }
}
