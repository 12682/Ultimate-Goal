package org.firstinspires.ftc.teamcode.tests.systems;

import com.goldenratiorobotics.robot.brain.controlprocessor.ControlProcessor;
import com.goldenratiorobotics.robot.brain.gyro.Gyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;

@TeleOp(name="Test DriveTrain", group="C. Tests System")
//@Disabled
public class TestDriveTrain extends LinearOpMode {

    private ElapsedTime runtime    = new ElapsedTime();
    private DriveTrain  driveTrain = null;
    private Gyro        gyro       = null;

    private double rotX         = 0;
    private double moveY        = 0;
    private double moveX        = 0;

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap);
        gyro       = new Gyro(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //region move trig
            rotX  = ControlProcessor.squareRoot(gamepad1.left_stick_x);
            moveY = ControlProcessor.cubeRoot(-gamepad1.left_stick_y);
            moveX = ControlProcessor.cubeRoot(gamepad1.right_stick_x);
            driveTrain.moveTrig(moveX, moveY, rotX);
            //endregion

            //region move directional
            while (gamepad1.dpad_left) {
                driveTrain.moveLeft(.35);
            }
            while (gamepad1.dpad_up) {
                driveTrain.moveForward(.35);
            }
            while (gamepad1.dpad_right) {
                driveTrain.moveRight(.35);
            }
            while (gamepad1.dpad_down) {
                driveTrain.moveBackward(.35);
            }
            //endregion

            //region rotate
            if (gamepad1.right_bumper) {
                driveTrain.rotateRight(.5);
                while (gyro.composeAngleYaw() > -90) {
                    /* wait */
                }
                driveTrain.stop();
            }
            if (gamepad1.left_bumper) {
                driveTrain.rotateLeft(.5);
                while (gyro.composeAngleYaw() < 0) {
                    /* wait */
                }
                driveTrain.stop();
            }
            //region end

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("DegreeYaw", gyro.composeAngleYaw());
            telemetry.addData("RotX", rotX);
            telemetry.addData("MoveY", moveY);
            telemetry.addData("MoveX", moveX);
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
