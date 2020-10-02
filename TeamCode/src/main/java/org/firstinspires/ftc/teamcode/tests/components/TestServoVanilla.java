package org.firstinspires.ftc.teamcode.tests.components;

import com.goldenratiorobotics.robot.brain.controlprocessor.ControlProcessor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test Servo Basic", group="D. Tests Components")
//@Disabled
public class TestServoVanilla extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Servo       servo   = null;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            servo.setPosition(gamepad1.right_trigger);


            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
            //endregion
        }
    }
}
