package org.firstinspires.ftc.teamcode.tests.components;

import com.goldenratiorobotics.robot.body.components.ServoNonContinuous;
import com.goldenratiorobotics.robot.brain.controlprocessor.ControlProcessor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test ServoNonContinuous Digital", group="D. Tests Components")
//@Disabled
public class TestServoNonContinuousDigital extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ServoNonContinuous servo   = null;

    private double minPWM;
    private double maxPWM;
    private double minPosition;
    private double maxPosition;
    private double position;

    @Override
    public void runOpMode() {
        servo = ServoNonContinuous.getInstance(hardwareMap, "servo");

        minPWM = 750;
        maxPWM = 2250;
        minPosition = (minPWM - 500) / 2000;
        maxPosition = (maxPWM - 500) / 2000;

        position = minPosition;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("min PWM", minPWM);
        telemetry.addData("max PWM", maxPWM);
        telemetry.addData("Min", minPosition);
        telemetry.addData("Max", maxPosition);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            sleep(1000);
            if (position == minPosition) {
                position = maxPosition;
            } else {
                position = minPosition;
            }

            servo.setPosition(position);

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
            //endregion
        }
    }
}
