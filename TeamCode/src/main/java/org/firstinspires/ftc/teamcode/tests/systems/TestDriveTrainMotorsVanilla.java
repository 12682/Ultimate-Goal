package org.firstinspires.ftc.teamcode.tests.systems;

import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test DriveTrain Motors Vanilla", group="C. Tests System")
//@Disabled
public class TestDriveTrainMotorsVanilla extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront   = null;
    private DcMotor leftBack    = null;
    private DcMotor rightFront  = null;
    private DcMotor rightBack   = null;

    private double speed        = .5;
    private int    encoderValueAll = 0;

    @Override
    public void runOpMode() {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                speed = Range.clip(speed - .1, -1, 1);
            }
            if (gamepad1.right_bumper) {
                speed = Range.clip(speed + .1, -1, 1);
            }
            if (gamepad1.a) {
                speed = -1 * speed;
            }

            while (gamepad1.dpad_down) {
                rightBack.setPower(speed);
            }
            while (gamepad1.dpad_right) {
                rightFront.setPower(speed);
            }
            while (gamepad1.dpad_up) {
                leftFront.setPower(speed);
            }
            while (gamepad1.dpad_left) {
                leftBack.setPower(speed);
            }

            rightBack.setPower(0);
            rightFront.setPower(0);
            leftFront.setPower(0);
            leftBack.setPower(0);

            encoderValueAll = (rightFront.getCurrentPosition() + rightBack.getCurrentPosition() + leftFront.getCurrentPosition() + leftBack.getCurrentPosition()) / 4;

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Selected Speed", speed);
            telemetry.addData("All Encoder Value", encoderValueAll);
            telemetry.addData("Left Front Encoder Value", leftFront.getCurrentPosition());
            telemetry.addData("Right Front Encoder Value", rightFront.getCurrentPosition());
            telemetry.addData("Left Back Encoder Value", leftBack.getCurrentPosition());
            telemetry.addData("Right Back Encoder Value", rightBack.getCurrentPosition());
            telemetry.update();
            //endregion
        }
    }
}
