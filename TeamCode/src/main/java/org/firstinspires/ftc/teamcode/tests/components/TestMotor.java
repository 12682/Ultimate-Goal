package org.firstinspires.ftc.teamcode.tests.components;

import com.goldenratiorobotics.robot.body.components.Motor;
import com.goldenratiorobotics.robot.brain.controlprocessor.ControlProcessor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test Motor", group="D. Tests Components")
//@Disabled
public class TestMotor extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Motor       motor   = null;

    private double speed        = 0;
    private int    encoderValue = 0;

    @Override
    public void runOpMode() {
        motor = Motor.getInstance(hardwareMap, "motor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            speed = gamepad1.right_trigger - gamepad1.left_trigger;

            motor.setSpeed(ControlProcessor.cubeRoot(speed));

            encoderValue = motor.getPosition();

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Encoder", encoderValue);
            telemetry.update();
            //endregion
        }
    }
}
