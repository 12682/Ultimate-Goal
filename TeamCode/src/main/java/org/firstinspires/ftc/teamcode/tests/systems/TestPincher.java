package org.firstinspires.ftc.teamcode.tests.systems;

import com.goldenratiorobotics.robot.body.pincher.Pincher;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test Pincher", group="C. Tests System")
//@Disabled
public class TestPincher extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Pincher     pincher = null;

    private double speed         = .5;
    private int    encoderValue  = 0;
    private float  servoPosition = 0;

    @Override
    public void runOpMode() {
        pincher = new Pincher(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            speed = (gamepad1.left_stick_y) / 2;
            servoPosition = gamepad1.right_trigger;

            pincher.moveServoManual(servoPosition);
            pincher.runMotorManual(speed);

            encoderValue = pincher.getMotorPosition();

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Selected Speed", speed);
            telemetry.addData("Motor Encoder", encoderValue);
            telemetry.addData("Servo Degree", pincher.getServoPosition());
            telemetry.addData("Servo Max", pincher.getServoMax());
            telemetry.addData("Servo Min", pincher.getServoMin());
            telemetry.addData("Pivot is Back", pincher.isPivotBack());
            telemetry.addData("Pivot is Lowered", pincher.isPivotLowered());
            telemetry.update();
            //endregion
        }
    }
}
