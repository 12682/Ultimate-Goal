package org.firstinspires.ftc.teamcode.teleop;

import com.goldenratiorobotics.robot.body.claimer.Claimer;
import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.latcher.Latcher;
import com.goldenratiorobotics.robot.body.pincher.Pincher;
import com.goldenratiorobotics.robot.brain.controlprocessor.ControlProcessor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp Mode", group="A. ModeTeleOp")
//@Disabled
public class ModeTeleOp extends LinearOpMode {

    private ElapsedTime    runtime     = new ElapsedTime();
    private DriveTrain     driveTrain  = null;
    private Pincher        pincher     = null;
    private Latcher        latcher     = null;
    private Claimer        claimer     = null;

    private double rotX          = 0;
    private double moveY         = 0;
    private double moveX         = 0;
    private double speedModifier = 0;
    private double multRot       = .33;
    private double multMoveY     = .50;
    private double multMoveX     = .70;

    private double pivotSpeed = 0;
    private double multPivot  = .33;

    @Override
    public void runOpMode() {
        driveTrain  = new DriveTrain(hardwareMap);
        pincher     = new Pincher(hardwareMap);
        latcher     = new Latcher(hardwareMap);
        claimer     = new Claimer(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0) {
                speedModifier = .4;
            } else {
                speedModifier = gamepad1.right_trigger - gamepad1.left_trigger;
            }

            rotX  = ControlProcessor.squareRoot(gamepad1.left_stick_x * speedModifier);
            moveY = ControlProcessor.cubeRoot( -gamepad1.left_stick_y * speedModifier);
            moveX = ControlProcessor.cubeRoot(gamepad1.right_stick_x * speedModifier);

            driveTrain.moveTrig(moveX, moveY, rotX);

            if (gamepad1.dpad_up) {
                driveTrain.moveForward(.40);
            }
            if (gamepad1.dpad_down) {
                driveTrain.moveBackward(.40);
            }
            if (gamepad1.dpad_left) {
                driveTrain.moveLeft(.40);
            }
            if (gamepad1.dpad_right) {
                driveTrain.moveRight(.40);
            }


            if (gamepad1.y) {
                latcher.unlatch();
            }
            if (gamepad1.a) {
                latcher.latch();
            }

            pivotSpeed = ControlProcessor.power(gamepad2.left_stick_y * multPivot, 2.0 / 3.0);

            pincher.runPivot(pivotSpeed);

            if (gamepad2.left_bumper) {
                pincher.release();
            }
            if (gamepad2.right_bumper) {
                pincher.pinch();
            }

            if (gamepad2.a) {
                if (gamepad2.right_trigger > .66) {
                    claimer.claim();
                } else {
                    claimer.neuter();
                }
            }

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed Mode", speedModifier);
            telemetry.update();
            //endregion
        }
    }
}
