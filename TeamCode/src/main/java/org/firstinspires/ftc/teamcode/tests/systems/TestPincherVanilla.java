package org.firstinspires.ftc.teamcode.tests.systems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test Pincher Basic", group="C. Tests System")
//@Disabled
public class TestPincherVanilla extends LinearOpMode {

    private ElapsedTime runtime      = new ElapsedTime();
    private Servo       pincherServo = null;
    private DcMotor     pivotMotor   = null;

    private double speed         = 0.5;
    private double servoPosition = 0.00;
    private double servoSpeed    = 0.1;
    private double servoMin      = 0.00;
    private double servoMax      = 1.00;

    @Override
    public void runOpMode() {
        pincherServo = hardwareMap.get(Servo.class, "pincherServo");
        pivotMotor   = hardwareMap.get(DcMotor.class, "pivotMotor");

        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            speed = (gamepad1.left_stick_y) / 2;
            pivotMotor.setPower(speed);

            if (gamepad1.dpad_up) {
                pincherServo.setPosition(1);
            }
            if (gamepad1.dpad_down) {
                pincherServo.setPosition(0);
            }
            pincherServo.setPosition(servoPosition);

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor Speed", pivotMotor.getPower());
            telemetry.addData("Motor Encoder", pivotMotor.getCurrentPosition());
            telemetry.addData("Servo Degree", pincherServo.getPosition());
            telemetry.update();
            //endregion
        }
    }
}
