package org.firstinspires.ftc.teamcode.tests.systems;

import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test DriveTrain Motors", group="C. Tests System")
//@Disabled
public class TestDriveTrainMotors extends LinearOpMode {

    private ElapsedTime runtime    = new ElapsedTime();
    private DriveTrain  driveTrain = null;

    private double speed        = .5;
    private int    encoderValue = 0;
    private String encoderType  = "None";

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap);

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
                driveTrain.runRightBack(speed);
            }
            while (gamepad1.dpad_right) {
                driveTrain.runRightFront(speed);
            }
            while (gamepad1.dpad_up) {
                driveTrain.runLeftFront(speed);
            }
            while (gamepad1.dpad_left) {
                driveTrain.runLeftBack(speed);
            }

            driveTrain.runAll(0);

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Selected Speed", speed);
            telemetry.addData("All Encoder Value", driveTrain.getPositionAll());
            telemetry.addData("Left Front Encoder Value", driveTrain.getPositionLeftFront());
            telemetry.addData("Right Front Encoder Value", driveTrain.getPositionRightFront());
            telemetry.addData("Left Back Encoder Value", driveTrain.getPositionLeftBack());
            telemetry.addData("Right Back Encoder Value", driveTrain.getPositionRightBack());
            telemetry.update();
            //endregion
        }
    }
}
